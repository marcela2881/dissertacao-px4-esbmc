/**
 * @file harness.cpp — candidato 6 (IridiumSBD.cpp)
 *
 * MODULO PX4: src/drivers/telemetry/iridiumsbd/IridiumSBD.cpp
 * FUNCAO:     IridiumSBD::write(), linhas 635-690 no commit 8be2590c15
 *
 * SITUACAO UPSTREAM: corrigido no PX4 pelo commit 0c903d47d6 (PR #28599,
 * 16/09/2026). Nao abrir issue. Compilar com -DCORRECAO_UPSTREAM aplica
 * essa correcao (etapa vii: reverificacao no patch).
 *
 * ============================================================
 * MECANISMO (v5: cadeia causal corrigida, parecer A10)
 * ============================================================
 *   // linha 669: guarda de espaco
 *   if (SATCOM_TX_BUF_LEN - _tx_buf_write_idx - _packet_length < 0)
 *       _tx_buf_write_idx = 0;
 *   // linha 675
 *   _packet_length -= buflen;
 *   // linha 683
 *   memcpy(_tx_buf + _tx_buf_write_idx, buffer, buflen);
 *   _tx_buf_write_idx += buflen;
 *
 * A escrita fora dos limites e causada pela GUARDA: ela compara o espaco
 * livre com _packet_length (o que ainda falta da mensagem declarada no
 * header MAVLink), mas o memcpy copia buflen. Quando um fragmento traz mais
 * bytes do que o header declarou (buflen > _packet_length), a guarda passa
 * e o memcpy estoura _tx_buf[340].
 *
 * Consequencia do MESMO pressuposto nao verificado (buflen <= _packet_length):
 * _packet_length -= buflen tambem estoura (uint16_t). Esse underflow acontece
 * DEPOIS da guarda e nao e usado no indice desta chamada; ele nao e a causa
 * da escrita fora dos limites.
 *
 * A mensagem do commit upstream descreve o mesmo par de defeitos.
 *
 * ============================================================
 * ESTADO INICIAL (v5, parecer B9)
 * ============================================================
 * O v4 forcava o cenario com __VERIFIER_assume(second_buflen > _packet_length).
 * Na v5 essa suposicao saiu: o payload_len e os flags do header e o tamanho
 * do segundo fragmento sao livres, limitados apenas pela checagem do proprio
 * driver (buflen <= SATCOM_TX_BUF_LEN, linha 638). O verificador deriva o
 * estado critico.
 *
 * v5 (parecer B12): <sys/types.h> declara ssize_t, entao nem o ESBMC nem o
 * CBMC precisam mais de -D'ssize_t=long'.
 */

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <sys/types.h>

extern size_t nondet_size_t();
extern uint8_t nondet_uint8();

// ================== CONSTANTES REAIS (IridiumSBD.h) ==================
#define SATCOM_TX_BUF_LEN     340

// ================== ESTADO REAL (membros da classe IridiumSBD) ==================
static uint8_t  _tx_buf[SATCOM_TX_BUF_LEN] = {};
static int      _tx_buf_write_idx = 0;
static uint16_t _packet_length = 0;
static bool     _writing_mavlink_packet = false;

// ================== LOGICA REAL, IridiumSBD::write() (8be2590c15, linhas 635-690) ==================
// Omitidos: mutex, contadores de estatistica, VERBOSE_INFO e timestamps.
ssize_t iridiumsbd_write(const char *buffer, size_t buflen)
{
	if ((ssize_t)buflen > SATCOM_TX_BUF_LEN) {
		return -1; // PX4_ERROR
	}

	if (!_writing_mavlink_packet) {
		if (buflen < 3) {
			_packet_length = buflen;

		} else if ((unsigned char)buffer[0] == 253 && (buflen == 10)) { // mavlink 2
			const uint8_t payload_len = buffer[1];
			const uint8_t incompat_flags = buffer[2];
			_packet_length = payload_len + 12;
			_writing_mavlink_packet = true;

			if (incompat_flags & 0x1) { //signing
				_packet_length += 13;
			}

		} else if ((unsigned char)buffer[0] == 254 && (buflen == 6)) { // mavlink 1
			const uint8_t payload_len = buffer[1];
			_packet_length = payload_len + 8;
			_writing_mavlink_packet = true;

		} else {
			_packet_length = buflen;
		}
	}

#ifdef CORRECAO_UPSTREAM
	// 0c903d47d6: a guarda considera tambem buflen, e a subtracao satura em zero
	const int space_left = SATCOM_TX_BUF_LEN - _tx_buf_write_idx;

	if (space_left < (int)_packet_length || space_left < (int)buflen) {
		_tx_buf_write_idx = 0;
	}

	_packet_length = (buflen >= _packet_length) ? 0 : _packet_length - buflen;
#else
	// linha 669: guarda compara com _packet_length, nao com buflen
	if (SATCOM_TX_BUF_LEN - _tx_buf_write_idx - _packet_length < 0) {
		_tx_buf_write_idx = 0;
	}

	// linha 675
	_packet_length -= buflen;
#endif

	if (_packet_length == 0) {
		_writing_mavlink_packet = false;
	}

	// linha 683
	memcpy(_tx_buf + _tx_buf_write_idx, buffer, buflen);
	_tx_buf_write_idx += (int)buflen;

	return (ssize_t)buflen;
}

int main()
{
	// Primeira chamada: header MAVLink 2 (10 bytes) com payload_len e flags livres
	char header[10] = {(char)253, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	header[1] = (char)nondet_uint8();
	header[2] = (char)nondet_uint8();
	iridiumsbd_write(header, 10);

	// Segunda chamada: fragmento de tamanho livre (o driver so limita a 340)
	size_t second_buflen = nondet_size_t();
	__VERIFIER_assume(second_buflen > 0 && second_buflen <= SATCOM_TX_BUF_LEN);

	char second_buf[SATCOM_TX_BUF_LEN];
	memset(second_buf, 0, sizeof(second_buf));

	iridiumsbd_write(second_buf, second_buflen);

	// PROPRIEDADE: o indice de escrita nunca passa do fim do buffer.
	assert(_tx_buf_write_idx <= SATCOM_TX_BUF_LEN);

	return 0;
}
