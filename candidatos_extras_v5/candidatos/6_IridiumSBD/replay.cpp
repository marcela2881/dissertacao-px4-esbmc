/**
 * @file replay.cpp — candidato 6 (IridiumSBD.cpp)
 *
 * Replay do contraexemplo com AddressSanitizer (v5, pareceres B4 e C4).
 *
 * O valor do contraexemplo NAO e mais fixado a mao (o v4 tinha 334, a issue
 * 331 e o log 335). O rodar_tudo.sh extrai do log do ESBMC payload_len,
 * flags e second_buflen e passa como argumentos:
 *
 *     ./replay <payload_len> <incompat_flags> <second_buflen>
 *
 * Sem argumentos, usa payload_len = 5, flags = 0, second_buflen = 334.
 *
 * write() abaixo e a mesma logica do harness (linhas 635-690 do
 * IridiumSBD.cpp no 8be2590c15). _tx_buf e global, como membro de um objeto
 * estatico; o ASan instrumenta o fim do global e acusa global-buffer-overflow.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>

#define SATCOM_TX_BUF_LEN 340

static uint8_t _tx_buf[SATCOM_TX_BUF_LEN];
static int _tx_buf_write_idx = 0;
static uint16_t _packet_length = 0;
static bool _writing_mavlink_packet = false;

static ssize_t iridiumsbd_write(const char *buffer, size_t buflen)
{
	if ((ssize_t)buflen > SATCOM_TX_BUF_LEN) {
		return -1;
	}

	if (!_writing_mavlink_packet) {
		if (buflen < 3) {
			_packet_length = buflen;

		} else if ((unsigned char)buffer[0] == 253 && (buflen == 10)) {
			const uint8_t payload_len = buffer[1];
			const uint8_t incompat_flags = buffer[2];
			_packet_length = payload_len + 12;
			_writing_mavlink_packet = true;

			if (incompat_flags & 0x1) {
				_packet_length += 13;
			}

		} else if ((unsigned char)buffer[0] == 254 && (buflen == 6)) {
			const uint8_t payload_len = buffer[1];
			_packet_length = payload_len + 8;
			_writing_mavlink_packet = true;

		} else {
			_packet_length = buflen;
		}
	}

	if (SATCOM_TX_BUF_LEN - _tx_buf_write_idx - _packet_length < 0) {
		_tx_buf_write_idx = 0;
	}

	_packet_length -= buflen;

	if (_packet_length == 0) {
		_writing_mavlink_packet = false;
	}

	memcpy(_tx_buf + _tx_buf_write_idx, buffer, buflen);
	_tx_buf_write_idx += (int)buflen;
	return (ssize_t)buflen;
}

int main(int argc, char **argv)
{
	int payload_len = (argc > 1) ? atoi(argv[1]) : 5;
	int flags = (argc > 2) ? atoi(argv[2]) : 0;
	int second = (argc > 3) ? atoi(argv[3]) : 334;

	char header[10] = {(char)253, (char)payload_len, (char)flags, 0, 0, 0, 0, 0, 0, 0};
	iridiumsbd_write(header, 10);
	printf("payload_len=%d flags=%d -> apos o header: _packet_length=%d _tx_buf_write_idx=%d\n",
	       payload_len, flags, _packet_length, _tx_buf_write_idx);

	static char second_buf[SATCOM_TX_BUF_LEN];
	memset(second_buf, 'A', sizeof(second_buf));
	printf("segundo fragmento: buflen=%d (indice final seria %d; buffer tem %d)\n",
	       second, _tx_buf_write_idx + second, SATCOM_TX_BUF_LEN);
	iridiumsbd_write(second_buf, (size_t)second);

	printf("Se chegou aqui: nenhuma deteccao. _tx_buf_write_idx=%d\n", _tx_buf_write_idx);
	return 0;
}
