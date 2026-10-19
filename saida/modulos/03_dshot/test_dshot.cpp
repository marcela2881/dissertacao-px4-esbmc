/*
 * Harness de verificação — telemetria serial do driver DShot do PX4
 * (src/drivers/dshot: DShotTelemetry.cpp, DShotCommon.h, esc/AM32Settings.cpp)
 *
 * O código NÃO é copiado: o harness inclui os .cpp ORIGINAIS do PX4 (commit
 * fixado). Stubs (saida/stubs): device::Serial (UART), hrt (tempo),
 * drv_dshot.h (só os códigos de comando), tópicos uORB esc_status/esc_eeprom_read
 * e uORB::Publication. Recorte: recepção da telemetria serial do ESC (quadro de
 * 10 bytes) e da resposta de configuração do AM32 (ESC_INFO, 48 + 1 bytes).
 * A classe DShot (saída de PWM digital, hardware up_dshot_*) não entra.
 *
 * Os campos privados de DShotTelemetry são acessados pelo harness
 * ("#define private public" só para a inclusão dos arquivos do PX4) para
 * partir de QUALQUER estado interno (testes por indução, como no tap_esc).
 *
 * PRÉ-CONDIÇÕES (origem anotada):
 *   - DIRECT_PWM_OUTPUT_CHANNELS = 8 (placas FMU v5/v6; board_config.h)
 *     -> DSHOT_MAX_MOTORS = min(8, CONNECTED_ESC_MAX = 12) = 8 (DShotCommon.h)
 *   - read() de parseTelemetryPacket pede sizeof(buf) = 10 bytes e o de
 *     parseCommandResponse pede 49 (DShotTelemetry.cpp); a UART devolve de -1
 *     até esse tamanho.
 *   - estado do leitor de telemetria: 0 <= _frame_position < 10 (é zerado ao
 *     completar o quadro, no timeout e em startTelemetryRequest).
 *   - estado da resposta de comando: 0 <= _command_response_position <= 49
 *     (zerado em reset/setExpectCommandResponse; teste do overflow antes do memcpy).
 *
 * PROPRIEDADES (do próprio protocolo, comentário "ESC Telemetry Frame Structure"
 * em DShotTelemetry.cpp, e de msg/EscEepromRead.msg + AM32Settings.cpp):
 *   - DP1: leitores não acessam fora de _frame_buffer[10] / _command_response_buffer[49]
 *          e o estado continua válido (invariante) após cada leitura da UART.
 *   - DP2: quadro com CRC8 correto -> Ready, e os campos são os do formato:
 *          temperatura = byte 0 [°C]; tensão = bytes 1-2 (big-endian) x 0,01 V;
 *          corrente = bytes 3-4 x 0,01 A; eRPM = bytes 7-8 x 100 (sem estouro, >= 0).
 *   - DP3: quadro com CRC8 errado -> ParseError e esc_data não é alterado.
 *   - DP4: resposta AM32 só é aceita com 49 bytes e CRC8 correto; nesse caso o
 *          tópico publicado traz os 48 bytes recebidos, length 48 e o índice do ESC.
 */

#include <cstdint>
#include <cstring>
#include <cassert>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

extern "C" {
int      nondet_int();
long     nondet_long();
uint8_t  nondet_uint8();
uint16_t nondet_uint16();
uint64_t nondet_uint64();
void     __VERIFIER_assume(int);   /* aceito pelo ESBMC e pelo CBMC */
}

#define DIRECT_PWM_OUTPUT_CHANNELS 8

#include <drivers/drv_hrt.h>

/* variáveis dos stubs (Serial.hpp, drv_hrt.h) */
bool          g_serial_fixed = false;
unsigned char g_serial_bytes[64];
long          g_serial_len = 0;
hrt_abstime   g_hrt_elapsed = 0;

/* arquivos ORIGINAIS do PX4 (unidade de tradução única) */
#define private public
#include "DShotTelemetry.cpp"
#include "esc/AM32Settings.cpp"
#undef private

#define FRAME 10                       /* DShotTelemetry::TELEMETRY_FRAME_SIZE */
#define CMD_MAX 49                     /* DShotTelemetry::COMMAND_RESPONSE_MAX_SIZE */

static void tempo_de_leitura()
{
	/* parseTelemetryPacket só lê depois de 800 us; qualquer tempo a partir daí */
	g_hrt_elapsed = nondet_uint64();
	__VERIFIER_assume(g_hrt_elapsed >= 800);
}

/* ---------------------------------------------------------------------------
 * D0 — DP1 no leitor de telemetria: parseTelemetryPacket a partir de QUALQUER
 * estado válido (_frame_position em [0,10), pedido em andamento), com a UART
 * devolvendo de -1 a 10 bytes quaisquer. Invariante mantido => vale para
 * qualquer sequência de leituras (indução).
 * ------------------------------------------------------------------------- */
void test_telemetry_reader()
{
	DShotTelemetry t;
	t._frame_position = nondet_int();
	__VERIFIER_assume(t._frame_position >= 0 && t._frame_position < FRAME);
	t._telemetry_request_start = 1;            /* pedido em andamento */
	tempo_de_leitura();

	EscData esc{};
	t.parseTelemetryPacket(&esc);

	assert(t._frame_position >= 0 && t._frame_position < FRAME);
}

/* quadro de 10 bytes livres; o byte 9 recebe o CRC8 dos 9 primeiros (correto)
 * ou um valor diferente dele (errado) */
static void monta_quadro(bool crc_correto)
{
	for (int i = 0; i < FRAME - 1; i++) { g_serial_bytes[i] = nondet_uint8(); }

	uint8_t crc = crc8(g_serial_bytes, FRAME - 1);

	if (crc_correto) {
		g_serial_bytes[FRAME - 1] = crc;

	} else {
		g_serial_bytes[FRAME - 1] = nondet_uint8();
		__VERIFIER_assume(g_serial_bytes[FRAME - 1] != crc);
	}

	g_serial_fixed = true;
	g_serial_len = FRAME;
}

/* ---------------------------------------------------------------------------
 * D1 — DP2: quadro completo com CRC correto, recebido de uma vez.
 * ------------------------------------------------------------------------- */
void test_telemetry_fields()
{
	DShotTelemetry t;
	t.startTelemetryRequest();
	t._telemetry_request_start = 1;            /* hrt_absolute_time() do stub = 0 */
	tempo_de_leitura();
	monta_quadro(true);
	const uint8_t *b = g_serial_bytes;

	EscData esc{};
	TelemetryStatus st = t.parseTelemetryPacket(&esc);

	assert(st == TelemetryStatus::Ready);
	assert(esc.temperature == (float)b[0]);
	assert(esc.voltage == (float)(uint16_t)((b[1] << 8) | b[2]) * 0.01f);
	assert(esc.current == (float)(uint16_t)((b[3] << 8) | b[4]) * 0.01f);
	assert(esc.erpm == (int32_t)((b[7] << 8) | b[8]) * 100);
	assert(esc.erpm >= 0);
	assert(t.telemetryResponseFinished() && t._frame_position == 0);
}

/* ---------------------------------------------------------------------------
 * D2 — DP3: quadro completo com CRC errado.
 * ------------------------------------------------------------------------- */
void test_telemetry_bad_crc()
{
	DShotTelemetry t;
	t.startTelemetryRequest();
	t._telemetry_request_start = 1;
	tempo_de_leitura();
	monta_quadro(false);

	EscData esc{};
	esc.erpm = 12345; esc.temperature = 7.0f; esc.voltage = 3.0f; esc.current = 2.0f;
	int erros = t._num_checksum_errors;

	TelemetryStatus st = t.parseTelemetryPacket(&esc);

	assert(st == TelemetryStatus::ParseError);
	assert(esc.erpm == 12345 && esc.temperature == 7.0f && esc.voltage == 3.0f && esc.current == 2.0f);
	assert(t._num_checksum_errors == erros + 1);
}

/* ---------------------------------------------------------------------------
 * D3 — DP1 na resposta de comando: parseCommandResponse a partir de QUALQUER
 * estado válido (posição em [0,49], comando e índice de motor quaisquer, com
 * ou sem tratador AM32 para o motor), UART devolvendo de -1 a 49 bytes.
 * ------------------------------------------------------------------------- */
void test_command_response()
{
	DShotTelemetry t;
	t._command_response_position = nondet_int();
	__VERIFIER_assume(t._command_response_position >= 0 && t._command_response_position <= CMD_MAX);
	t._command_response_motor_index = nondet_int();
	t._command_response_command = nondet_uint16();
	t._command_response_start = 1;
	g_hrt_elapsed = nondet_uint64();           /* inclui o caminho de timeout (> 1 s) */

	int m = nondet_int();
	__VERIFIER_assume(m >= 0 && m < DSHOT_MAX_MOTORS);

	if (nondet_int()) { t._settings_handlers[m] = new AM32Settings(m); }

	t.parseCommandResponse();

	assert(t._command_response_position >= 0 && t._command_response_position <= CMD_MAX);
}

/* ---------------------------------------------------------------------------
 * D4 — DP4: AM32Settings::decodeInfoResponse com buffer de 49 bytes quaisquer
 * e tamanho qualquer em [0,49] (o chamador passa _command_response_position).
 * ------------------------------------------------------------------------- */
void test_am32_decode()
{
	int idx = nondet_int();
	__VERIFIER_assume(idx >= 0 && idx < DSHOT_MAX_MOTORS);
	AM32Settings s(idx);

	uint8_t buf[CMD_MAX];

	for (int i = 0; i < CMD_MAX; i++) { buf[i] = nondet_uint8(); }

	int size = nondet_int();
	__VERIFIER_assume(size >= 0 && size <= CMD_MAX);

	AM32Settings::_esc_eeprom_read_pub.published = false;
	bool ok = s.decodeInfoResponse(buf, size);

	bool esperado = (size == EEPROM_SIZE + 1) && (crc8(buf, EEPROM_SIZE) == buf[EEPROM_SIZE]);
	assert(ok == esperado);

	if (ok) {
		const esc_eeprom_read_s &d = AM32Settings::_esc_eeprom_read_pub.last;
		assert(AM32Settings::_esc_eeprom_read_pub.published);
		assert(d.length == EEPROM_SIZE && d.index == idx && d.firmware == 1);

		for (int i = 0; i < EEPROM_SIZE; i++) { assert(d.data[i] == buf[i]); }

	} else {
		assert(!AM32Settings::_esc_eeprom_read_pub.published);
	}
}

/* Ponto de entrada: --function harness_main (mesmo padrão do RoboClaw). */
int harness_main()
{
#ifdef TEST_CHOICE
	int choice = TEST_CHOICE;                 /* rodar um teste por vez: -DTEST_CHOICE=n */
#else
	int choice = nondet_int();
	__VERIFIER_assume(choice >= 0 && choice <= 4);
#endif

	switch (choice) {
	case 0: test_telemetry_reader();  break;
	case 1: test_telemetry_fields();  break;
	case 2: test_telemetry_bad_crc(); break;
	case 3: test_command_response();  break;
	case 4: test_am32_decode();       break;
	}

	return 0;
}

int main() { return harness_main(); }
