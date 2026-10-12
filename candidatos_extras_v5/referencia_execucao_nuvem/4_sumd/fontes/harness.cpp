/**
 * @file harness.cpp — candidato 4 (sumd.cpp)
 *
 * MODULO PX4: src/lib/rc/sumd.cpp (sumd_decode) e sumd.h
 * BASE: PX4 commit 8be2590c15; o codigo e IGUAL no main de 12/10/2026.
 * METODO: Bounded Model Checking com ESBMC / CBMC (--unwind 70)
 *
 * Candidato #4 (v5: numeracao corrigida, parecer B22).
 * SUMD_MAX_CHANNELS = 32, sumd_data[64] (indices validos 0..63). O
 * decodificador aceita length ate 32. No estado GOT_LEN ele escreve
 * sumd_data[_rxlen] com _rxlen comecando em 1 e continua enquanto
 *     _rxlen <= _rxpacket.length * 2
 * (sumd.cpp linhas 196 e 207). Com length = 32 a 64a escrita cai em
 * sumd_data[64], um byte alem do array, que no struct empacotado e o campo
 * crc16_high. Um unico quadro legitimo de 32 canais basta.
 *
 * O que este harness copia: o switch de sumd_decode() ate o estado GOT_CRC.
 * O que ele OMITE (v5, correcao do comentario anterior que dizia
 * "transcricao verbatim"): os printf de debug (sempre desligados) e a
 * extracao de canais do estado GOT_CRC. A extracao e exercitada pelo
 * replay.cpp, que roda a funcao real inteira.
 *
 * Estado inicial: derivado da entrada (quadro com header, status e
 * length = 32 legitimos; bytes de dados livres). Nada assumido alem do
 * length maximo permitido pelo proprio decodificador.
 */

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

extern uint8_t nondet_uint8();

// ================== DEFINIÇÕES REAIS, extraídas de sumd.h ==================
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
	uint8_t	sumd_data[SUMD_MAX_CHANNELS * 2]; // sumd_data[64], índices válidos 0..63
	uint8_t	crc16_high;
	uint8_t	crc16_low;
	uint8_t	telemetry;
	uint8_t	crc8;
} ReceiverFcPacketHoTT;
#pragma pack(pop)

// ================== ESTADO REAL, extraído de sumd.cpp ==================
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

uint8_t		_crc8	= 0x00;
uint16_t	_crc16 = 0x0000;
bool		_sumd		= true;
bool		_crcOK		= false;

static enum SUMD_DECODE_STATE _decode_state = SUMD_DECODE_STATE_UNSYNCED;
static uint8_t _rxlen;
static ReceiverFcPacketHoTT _rxpacket; // simplificado: instância direta em vez da union real rc_decode_buf

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

// ================== FUNÇÃO REAL EXTRAÍDA de sumd.cpp linhas 111-367 ==================
// (printf de debug removidos; sempre guardados por "if (_debug)" == false)
int sumd_decode(uint8_t byte, uint8_t *rssi, uint8_t *rx_count, uint16_t *channel_count, uint16_t *channels,
		uint16_t max_chan_count, bool *failsafe)
{
	int ret = 1;

	switch (_decode_state) {
	case SUMD_DECODE_STATE_UNSYNCED:
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

		} else {
			_decode_state = SUMD_DECODE_STATE_UNSYNCED;
		}

		break;

	case SUMD_DECODE_STATE_GOT_LEN:
		// linha 196 de sumd.cpp: escrita SEM checagem de limite
		_rxpacket.sumd_data[_rxlen] = byte;

		if (_sumd) {
			_crc16 = sumd_crc16(_crc16, byte);

		} else {
			_crc8 = sumd_crc8(_crc8, byte);
		}

		_rxlen++;

		// linha 207 de sumd.cpp: condição "<=" (deveria ser "<")
		if (_rxlen <= ((_rxpacket.length * 2))) {
			// permanece no mesmo estado

		} else {
			_decode_state = SUMD_DECODE_STATE_GOT_DATA;
		}

		break;

	case SUMD_DECODE_STATE_GOT_DATA:
		_rxpacket.crc16_high = byte;

		if (_sumd) {
			_decode_state = SUMD_DECODE_STATE_GOT_CRC;

		} else {
			_decode_state = SUMD_DECODE_STATE_GOT_CRC16_BYTE_1;
		}

		break;

	case SUMD_DECODE_STATE_GOT_CRC16_BYTE_1:
		_rxpacket.crc16_low = byte;
		_decode_state = SUMD_DECODE_STATE_GOT_CRC16_BYTE_2;
		break;

	case SUMD_DECODE_STATE_GOT_CRC16_BYTE_2:
		_rxpacket.telemetry = byte;
		_decode_state = SUMD_DECODE_STATE_GOT_CRC;
		break;

	case SUMD_DECODE_STATE_GOT_CRC:
		if (_sumd) {
			_rxpacket.crc16_low = byte;

			if (_crc16 == (uint16_t)(_rxpacket.crc16_high << 8) + _rxpacket.crc16_low) {
				_crcOK = true;
			}

		} else {
			_rxpacket.crc8 = byte;

			if (_crc8 == _rxpacket.crc8) {
				_crcOK = true;
			}
		}

		if (_crcOK) {
			ret = 0;
			uint8_t _cnt = *rx_count + 1;
			*rx_count = _cnt;
			*rssi = 100;
			*failsafe = (_rxpacket.status == SUMD_ID_FAILSAFE);

			if ((uint16_t)_rxpacket.length > max_chan_count) {
				_rxpacket.length = (uint8_t) max_chan_count;
			}

			*channel_count = (uint16_t)_rxpacket.length;

		} else {
			ret = 4;
		}

		_decode_state = SUMD_DECODE_STATE_UNSYNCED;
		break;
	}

	return ret;
}

// ================== HARNESS ==================
int main()
{
	uint8_t rssi = 0, rx_count = 0;
	uint16_t channel_count = 0;
	uint16_t channels[SUMD_MAX_CHANNELS];
	bool failsafe = false;

	_decode_state = SUMD_DECODE_STATE_UNSYNCED;
	_rxlen = 0;

	// Byte 1: header válido (obrigatório para entrar no parser)
	sumd_decode(SUMD_HEADER_ID, &rssi, &rx_count, &channel_count, channels, SUMD_MAX_CHANNELS, &failsafe);

	// Byte 2: status válido (SUMD)
	sumd_decode(SUMD_ID_SUMD, &rssi, &rx_count, &channel_count, channels, SUMD_MAX_CHANNELS, &failsafe);

	// Byte 3: length = SUMD_MAX_CHANNELS (32) - valor MÁXIMO LEGÍTIMO,
	// aceito pela própria validação "byte <= SUMD_MAX_CHANNELS".
	sumd_decode(SUMD_MAX_CHANNELS, &rssi, &rx_count, &channel_count, channels, SUMD_MAX_CHANNELS, &failsafe);

	// Bytes de dados: alimentamos length*2 + 1 = 65 bytes simbólicos,
	// reproduzindo exatamente a sequência que, na análise manual,
	// resulta na escrita em sumd_data[64] (fora dos limites).
	for (int i = 0; i < (SUMD_MAX_CHANNELS * 2) + 1; i++) {
		uint8_t b = nondet_uint8();
		sumd_decode(b, &rssi, &rx_count, &channel_count, channels, SUMD_MAX_CHANNELS, &failsafe);
	}

	// PROPRIEDADE: _rxlen nunca deveria ultrapassar o tamanho real do
	// array sumd_data (64), refletindo que nenhuma escrita ocorreu
	// além de seus limites.
	assert(_rxlen <= SUMD_MAX_CHANNELS * 2);

	return 0;
}
