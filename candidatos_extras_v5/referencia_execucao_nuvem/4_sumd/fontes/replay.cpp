/**
 * @file replay.cpp — candidato 4 (sumd.cpp)
 *
 * v5: este replay executa a funcao sumd_decode() REAL. O trecho entre as
 * marcas "INICIO/FIM DO CODIGO REAL" foi copiado sem alteracao de
 * src/lib/rc/sumd.cpp (linhas 70-367, igual nos commits 8be2590c15 e
 * bf6603b42b). So o cabecalho (sumd.h, common_rc.h) foi substituido pelas
 * definicoes abaixo, copiadas de sumd.h.
 *
 * O replay v4 reimplementava o laco no main(); este substitui aquele.
 *
 * Entrada: um quadro SUMD valido de 32 canais (o maximo que o proprio
 * decodificador aceita), com CRC16 correto. Canal k tem o valor 0x1000+k*8.
 *
 *   ./replay 32   -> max_chan_count = 32 (como em rc_tests/RCTest.cpp)
 *   ./replay 18   -> max_chan_count = 18 (como em RCInput.cpp, RC_INPUT_MAX_CHANNELS)
 *
 * Com UBSan: "index 64 out of bounds for type ... [64]" na escrita
 * _rxpacket.sumd_data[_rxlen] = byte (estado GOT_LEN).
 * Sem sanitizador: mostra o efeito. O 64o byte de dados cai em crc16_high e
 * e sobrescrito pelo byte alto do CRC; com max_chan_count = 32 o canal 32
 * e decodificado com esse byte no lugar do seu byte baixo.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define SUMD_MAX_CHANNELS	32
#define SUMD_HEADER_LENGTH	3
#define SUMD_HEADER_ID		0xA8
#define SUMD_ID_SUMH		0x00
#define SUMD_ID_SUMD		0x01
#define SUMD_ID_FAILSAFE	0x81

#pragma pack(push, 1)
typedef struct {
	uint8_t	header;
	uint8_t	status;
	uint8_t	length;
	uint8_t	sumd_data[SUMD_MAX_CHANNELS * 2];
	uint8_t	crc16_high;
	uint8_t	crc16_low;
	uint8_t	telemetry;
	uint8_t	crc8;
} ReceiverFcPacketHoTT;
#pragma pack(pop)

/* No PX4 o pacote vive na union rc_decode_buf (common_rc.h). */
static ReceiverFcPacketHoTT _hottrxpacket_storage;
#define rc_decode_buf_hott _hottrxpacket_storage

enum SUMD_DECODE_STATE {
	SUMD_DECODE_STATE_UNSYNCED = 0,
	SUMD_DECODE_STATE_GOT_HEADER,
	SUMD_DECODE_STATE_GOT_STATE,
	SUMD_DECODE_STATE_GOT_LEN,
	SUMD_DECODE_STATE_GOT_DATA,
	SUMD_DECODE_STATE_GOT_CRC,
	SUMD_DECODE_STATE_GOT_CRC16_BYTE_1,
	SUMD_DECODE_STATE_GOT_CRC16_BYTE_2
};

/* ======================= INICIO DO CODIGO REAL ======================= */
uint8_t 	_crc8 	= 0x00;
uint16_t 	_crc16 = 0x0000;
bool 		_sumd 		= true;
bool		_crcOK		= false;
bool		_debug		= false;


/* define range mapping here, -+100% -> 1000..2000 */
#define SUMD_RANGE_MIN 0.0f
#define SUMD_RANGE_MAX 4096.0f

#define SUMD_TARGET_MIN 1000.0f
#define SUMD_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SUMD_SCALE_FACTOR ((SUMD_TARGET_MAX - SUMD_TARGET_MIN) / (SUMD_RANGE_MAX - SUMD_RANGE_MIN))
#define SUMD_SCALE_OFFSET (int)(SUMD_TARGET_MIN - (SUMD_SCALE_FACTOR * SUMD_RANGE_MIN + 0.5f))

static enum SUMD_DECODE_STATE _decode_state = SUMD_DECODE_STATE_UNSYNCED;
static uint8_t _rxlen;

static ReceiverFcPacketHoTT &_rxpacket = rc_decode_buf_hott; /* unica linha adaptada: rc_decode_buf._hottrxpacket */

uint16_t sumd_crc16(uint16_t crc, uint8_t value)
{
	int i;
	crc ^= (uint16_t)value << 8;

	for (i = 0; i < 8; i++) {
		crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
	}

	return crc;
}

uint8_t sumd_crc8(uint8_t crc, uint8_t value)
{
	crc += value;
	return crc;
}

int sumd_decode(uint8_t byte, uint8_t *rssi, uint8_t *rx_count, uint16_t *channel_count, uint16_t *channels,
		uint16_t max_chan_count, bool *failsafe)
{

	int ret = 1;

	switch (_decode_state) {
	case SUMD_DECODE_STATE_UNSYNCED:
		if (_debug) {
			printf(" SUMD_DECODE_STATE_UNSYNCED \n") ;
		}

		if (byte == SUMD_HEADER_ID) {
			_rxpacket.header = byte;
			_sumd = true;
			_rxlen = 0;
			_crc16 = 0x0000;
			_crc8 = 0x00;
			_crcOK = false;
			_crc16 = sumd_crc16(_crc16, byte);
			_crc8 = sumd_crc8(_crc8, byte);
			_decode_state = SUMD_DECODE_STATE_GOT_HEADER;

			if (_debug) {
				printf(" SUMD_DECODE_STATE_GOT_HEADER: %x \n", byte) ;
			}

		} else {
			ret = 3;
		}

		break;

	case SUMD_DECODE_STATE_GOT_HEADER:
		if (byte == SUMD_ID_SUMD || byte == SUMD_ID_FAILSAFE || byte == SUMD_ID_SUMH) {
			_rxpacket.status = byte;

			if (byte == SUMD_ID_SUMH) {
				_sumd = false;
			}

			if (_sumd) {
				_crc16 = sumd_crc16(_crc16, byte);

			} else {
				_crc8 = sumd_crc8(_crc8, byte);
			}

			_decode_state = SUMD_DECODE_STATE_GOT_STATE;

			if (_debug) {
				printf(" SUMD_DECODE_STATE_GOT_STATE: %x \n", byte) ;
			}

		} else {
			_decode_state = SUMD_DECODE_STATE_UNSYNCED;
		}

		break;

	case SUMD_DECODE_STATE_GOT_STATE:
		if (byte >= 2 && byte <= SUMD_MAX_CHANNELS) {
			_rxpacket.length = byte;

			if (_sumd) {
				_crc16 = sumd_crc16(_crc16, byte);

			} else {
				_crc8 = sumd_crc8(_crc8, byte);
			}

			_rxlen++;
			_decode_state = SUMD_DECODE_STATE_GOT_LEN;

			if (_debug) {
				printf(" SUMD_DECODE_STATE_GOT_LEN: %x (%d) \n", byte, byte) ;
			}

		} else {
			_decode_state = SUMD_DECODE_STATE_UNSYNCED;
		}

		break;

	case SUMD_DECODE_STATE_GOT_LEN:
		_rxpacket.sumd_data[_rxlen] = byte;

		if (_sumd) {
			_crc16 = sumd_crc16(_crc16, byte);

		} else {
			_crc8 = sumd_crc8(_crc8, byte);
		}

		_rxlen++;

		if (_rxlen <= ((_rxpacket.length * 2))) {
			if (_debug) {
				printf(" SUMD_DECODE_STATE_GOT_DATA[%d]: %x\n", _rxlen - 2, byte) ;
			}

		} else {
			_decode_state = SUMD_DECODE_STATE_GOT_DATA;

			if (_debug) {
				printf(" SUMD_DECODE_STATE_GOT_DATA -- finish --\n") ;
			}

		}

		break;

	case SUMD_DECODE_STATE_GOT_DATA:
		_rxpacket.crc16_high = byte;

		if (_debug) {
			printf(" SUMD_DECODE_STATE_GOT_CRC16[1]: %x   [%x]\n", byte, ((_crc16 >> 8) & 0xff)) ;
		}

		if (_sumd) {
			_decode_state = SUMD_DECODE_STATE_GOT_CRC;

		} else {
			_decode_state = SUMD_DECODE_STATE_GOT_CRC16_BYTE_1;
		}

		break;

	case SUMD_DECODE_STATE_GOT_CRC16_BYTE_1:
		_rxpacket.crc16_low = byte;

		if (_debug) {
			printf(" SUMD_DECODE_STATE_GOT_CRC16[2]: %x   [%x]\n", byte, (_crc16 & 0xff)) ;
		}

		_decode_state = SUMD_DECODE_STATE_GOT_CRC16_BYTE_2;

		break;

	case SUMD_DECODE_STATE_GOT_CRC16_BYTE_2:
		_rxpacket.telemetry = byte;

		if (_debug) {
			printf(" SUMD_DECODE_STATE_GOT_SUMH_TELEMETRY: %x\n", byte) ;
		}

		_decode_state = SUMD_DECODE_STATE_GOT_CRC;

		break;

	case SUMD_DECODE_STATE_GOT_CRC:
		if (_sumd) {
			_rxpacket.crc16_low = byte;

			if (_debug) {
				printf(" SUMD_DECODE_STATE_GOT_CRC[2]: %x   [%x]\n\n", byte, (_crc16 & 0xff)) ;
			}

			if (_crc16 == (uint16_t)(_rxpacket.crc16_high << 8) + _rxpacket.crc16_low) {
				_crcOK = true;
			}

		} else {
			_rxpacket.crc8 = byte;

			if (_debug) {
				printf(" SUMD_DECODE_STATE_GOT_CRC8_SUMH: %x   [%x]\n\n", byte, _crc8) ;
			}

			if (_crc8 == _rxpacket.crc8) {
				_crcOK = true;
			}
		}

		if (_crcOK) {
			if (_debug) {
				printf(" CRC - OK \n") ;
			}

			if (_sumd) {
				if (_debug) {
					printf(" Got valid SUMD Packet\n") ;
				}

			} else {
				if (_debug) {
					printf(" Got valid SUMH Packet\n") ;
				}

			}

			if (_debug) {
				printf(" RXLEN: %d  [Chans: %d] \n\n", _rxlen - 1, (_rxlen - 1) / 2) ;
			}

			ret = 0;
			unsigned i;
			uint8_t _cnt = *rx_count + 1;
			*rx_count = _cnt;

			*rssi = 100;

			/* failsafe flag */
			*failsafe = (_rxpacket.status == SUMD_ID_FAILSAFE);

			/* received Channels */
			if ((uint16_t)_rxpacket.length > max_chan_count) {
				_rxpacket.length = (uint8_t) max_chan_count;
			}

			*channel_count = (uint16_t)_rxpacket.length;

			/* decode the actual packet */
			/* reorder first 4 channels */

			/* ch1 = roll -> sumd = ch2 */
			channels[0] = (uint16_t)((_rxpacket.sumd_data[1 * 2 + 1] << 8) | _rxpacket.sumd_data[1 * 2 + 2]) >> 3;
			/* ch2 = pitch -> sumd = ch2 */
			channels[1] = (uint16_t)((_rxpacket.sumd_data[2 * 2 + 1] << 8) | _rxpacket.sumd_data[2 * 2 + 2]) >> 3;
			/* ch3 = throttle -> sumd = ch2 */
			channels[2] = (uint16_t)((_rxpacket.sumd_data[0 * 2 + 1] << 8) | _rxpacket.sumd_data[0 * 2 + 2]) >> 3;
			/* ch4 = yaw -> sumd = ch2 */
			channels[3] = (uint16_t)((_rxpacket.sumd_data[3 * 2 + 1] << 8) | _rxpacket.sumd_data[3 * 2 + 2]) >> 3;

			/* we start at channel 5(index 4) */
			unsigned chan_index = 4;

			for (i = 4; i < _rxpacket.length; i++) {
				if (_debug) {
					printf("ch[%d] : %x %x [ %x    %d ]\n", i + 1, _rxpacket.sumd_data[i * 2 + 1], _rxpacket.sumd_data[i * 2 + 2],
					       ((_rxpacket.sumd_data[i * 2 + 1] << 8) | _rxpacket.sumd_data[i * 2 + 2]) >> 3,
					       ((_rxpacket.sumd_data[i * 2 + 1] << 8) | _rxpacket.sumd_data[i * 2 + 2]) >> 3);
				}

				channels[chan_index] = (uint16_t)((_rxpacket.sumd_data[i * 2 + 1] << 8) | _rxpacket.sumd_data[i * 2 + 2]) >> 3;
				/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
				//channels[chan_index] = (uint16_t)(channels[chan_index] * SUMD_SCALE_FACTOR + .5f) + SUMD_SCALE_OFFSET;

				chan_index++;
			}

		} else {
			/* decoding failed */
			ret = 4;

			if (_debug) {
				printf(" CRC - fail \n") ;
			}

		}

		_decode_state = SUMD_DECODE_STATE_UNSYNCED;
		break;
	}

	return ret;
}
/* ======================== FIM DO CODIGO REAL ========================= */

int main(int argc, char **argv)
{
	unsigned max_chan = (argc > 1) ? (unsigned)atoi(argv[1]) : 32;
	uint8_t frame[3 + 64 + 2];
	frame[0] = SUMD_HEADER_ID;
	frame[1] = SUMD_ID_SUMD;
	frame[2] = 32;

	for (int ch = 0; ch < 32; ch++) {
		uint16_t v = (uint16_t)(0x1000 + ch * 8);
		frame[3 + ch * 2] = (uint8_t)(v >> 8);
		frame[3 + ch * 2 + 1] = (uint8_t)(v & 0xFF);
	}

	uint16_t crc = 0;
	for (int i = 0; i < 3 + 64; i++) { crc = sumd_crc16(crc, frame[i]); }
	frame[67] = (uint8_t)(crc >> 8);
	frame[68] = (uint8_t)(crc & 0xFF);

	uint8_t rssi = 0, rx_count = 0;
	uint16_t channel_count = 0;
	uint16_t channels[32] = {0};
	bool failsafe = false;
	int ret = -1;

	for (unsigned i = 0; i < sizeof(frame); i++) {
		ret = sumd_decode(frame[i], &rssi, &rx_count, &channel_count, channels, (uint16_t)max_chan, &failsafe);
	}

	printf("max_chan_count=%u ret=%d (0 = frame decoded) channel_count=%u\n", max_chan, ret, channel_count);
	printf("frame CRC16 = 0x%04X (high byte 0x%02X)\n", crc, crc >> 8);

	if (channel_count == 32) {
		uint16_t esperado = (uint16_t)((0x1000 + 31 * 8) >> 3);
		printf("channel 32: decoded=%u expected=%u (low byte sent = 0x%02X, CRC high byte = 0x%02X)\n",
		       channels[31], esperado, (unsigned)frame[3 + 63], (unsigned)(crc >> 8));
	} else {
		printf("channel 32 is not decoded with max_chan_count=%u\n", max_chan);
	}
	return 0;
}
