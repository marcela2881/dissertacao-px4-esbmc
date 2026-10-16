/*
 * Harness de verificação — protocolo UART do voxl_esc do PX4
 * (src/drivers/actuators/voxl_esc: qc_esc_packet.c e crc16.c)
 *
 * O código NÃO é copiado: o harness inclui os arquivos .c ORIGINAIS do PX4
 * (commit fixado). Esses arquivos não dependem de NuttX/uORB, então nenhum
 * stub é necessário. Recorte: a biblioteca de pacotes (montagem e leitura dos
 * pacotes trocados com o ESC); a classe VoxlEsc em si não entra neste harness.
 *
 * PRÉ-CONDIÇÕES (origem anotada):
 *   - fb_id em [0, 3] ou -1: o driver alterna _fb_idx entre os 4 ESCs
 *     (voxl_esc.cpp) e as funções sem _fb usam -1.
 *   - Nos demais testes as entradas são livres (qualquer valor do tipo).
 *
 * PROPRIEDADES DE ESPECIFICAÇÃO (protocolo definido no próprio
 * qc_esc_packet.h; a conferir com a documentação da ModalAI, se houver):
 *   - P1: um pacote montado por qc_esc_create_packet é aceito pelo leitor
 *         qc_esc_packet_process_char (formato e CRC consistentes).
 *   - P2: o comando de RPM enviado é o pedido, saturado em
 *         [ESC_RPM_MIN, ESC_RPM_MAX] (ou /2 no modo estendido), sem inverter
 *         o sinal por estouro de int16 (comentário no código: "Limit RPMs to
 *         prevent overflow when converting to int16_t").
 *   - P3: o comando PWM enviado fica em [-800, 800] e não inverte o sinal.
 *   - P4: só o canal fb_id leva o bit 0 ligado (pedido de feedback).
 */

#include <stdint.h>
#include <string.h>
#include <assert.h>

/* arquivos ORIGINAIS do PX4 (unidade de tradução única) */
#include "crc16.c"
#include "qc_esc_packet.c"

int      nondet_int(void);
int16_t  nondet_int16(void);
int32_t  nondet_int32(void);
uint8_t  nondet_uint8(void);
uint16_t nondet_uint16(void);
void     __VERIFIER_assume(int);   /* aceito pelo ESBMC e pelo CBMC */

#define OUT_SIZE 128                /* Command::BUF_SIZE em voxl_esc.hpp */
#define N_CHARS  70                 /* > 64 (buffer do leitor): cobre o reinício */

static int16_t decode_i16(const uint8_t *out, int k)
{
	int16_t v;
	memcpy(&v, &out[ESC_PACKET_POS_DATA + 2 * k], sizeof(v));
	return v;
}

/* ---------------------------------------------------------------------------
 * V0 — Leitor de pacotes com bytes quaisquer vindos da UART: segurança de
 * memória (ponteiro bp dentro de buffer[64]). Propriedades automáticas do ESBMC.
 * ------------------------------------------------------------------------- */
void test_parser_memory(void)
{
	EscPacket p;
	qc_esc_packet_init(&p);

	for (int i = 0; i < N_CHARS; i++) {
		qc_esc_packet_process_char(nondet_uint8(), &p);
		assert(p.len_received < sizeof(p.buffer));
	}
}

/* ---------------------------------------------------------------------------
 * V1 — P1: ida e volta (montar -> ler). Tamanho do payload limitado ao que o
 * leitor aceita (len_expected < 63 -> payload <= 57).
 * ------------------------------------------------------------------------- */
void test_roundtrip(void)
{
	uint8_t data[57];
	uint8_t out[OUT_SIZE];
	uint16_t n = nondet_uint16();
	__VERIFIER_assume(n <= 57);

	for (int i = 0; i < 57; i++) { data[i] = nondet_uint8(); }

	int32_t len = qc_esc_create_packet(nondet_uint8(), data, n, out, OUT_SIZE);
	assert(len == n + 5);

	EscPacket p;
	qc_esc_packet_init(&p);
	int16_t ret = ESC_NO_PACKET;

	for (int i = 0; i < len; i++) { ret = qc_esc_packet_process_char(out[i], &p); }

	assert(ret == len);                                   /* P1 */
}

/* ---------------------------------------------------------------------------
 * V2 — P2: comando de RPM, qualquer int32, modo normal ou estendido.
 * ------------------------------------------------------------------------- */
void test_rpm_command(void)
{
	int32_t rpm[4];
	uint8_t out[OUT_SIZE];
	uint8_t ext = nondet_uint8();

	for (int k = 0; k < 4; k++) { rpm[k] = nondet_int32(); }

	int32_t len = qc_esc_create_rpm_packet4_fb(rpm[0], rpm[1], rpm[2], rpm[3], 0, 0, 0, 0, -1,
			out, OUT_SIZE, ext);
	assert(len == 15);

	int32_t max = ext > 0 ? ESC_RPM_MAX_EXT : ESC_RPM_MAX;
	int32_t min = ext > 0 ? ESC_RPM_MIN_EXT : ESC_RPM_MIN;

	for (int k = 0; k < 4; k++) {
		int32_t r = rpm[k] > max ? max : (rpm[k] < min ? min : rpm[k]);
		int32_t esperado = (ext > 0) ? (r / 4) * 2 : r;
		esperado &= ~1;                                    /* bit 0 = pedido de feedback */
		int16_t enviado = decode_i16(out, k);
		assert(enviado == esperado);                       /* P2: sem truncamento */

		if (rpm[k] > 0) { assert(enviado >= 0); }          /* P2: sem inversão de sinal */

		if (rpm[k] < 0) { assert(enviado <= 0); }
	}
}

/* ---------------------------------------------------------------------------
 * V3 — P3: comando PWM, qualquer int16.
 * ------------------------------------------------------------------------- */
void test_pwm_command(void)
{
	int16_t pwm[4];
	uint8_t out[OUT_SIZE];

	for (int k = 0; k < 4; k++) { pwm[k] = nondet_int16(); }

	int32_t len = qc_esc_create_pwm_packet4_fb(pwm[0], pwm[1], pwm[2], pwm[3], 0, 0, 0, 0, -1,
			out, OUT_SIZE);
	assert(len == 15);

	for (int k = 0; k < 4; k++) {
		int16_t enviado = decode_i16(out, k);
		assert(enviado >= -800 && enviado <= 800);         /* P3 */

		if (pwm[k] > 0) { assert(enviado >= 0); }

		if (pwm[k] < 0) { assert(enviado <= 0); }
	}
}

/* ---------------------------------------------------------------------------
 * V4 — P4: bit de pedido de feedback só no canal fb_id (modo RPM e PWM).
 * ------------------------------------------------------------------------- */
void test_feedback_bit(void)
{
	uint8_t out[OUT_SIZE];
	int32_t fb_id = nondet_int32();
	__VERIFIER_assume(fb_id >= 0 && fb_id <= 3);          /* _fb_idx em voxl_esc.cpp */

	if (nondet_int()) {
		qc_esc_create_rpm_packet4_fb(nondet_int32(), nondet_int32(), nondet_int32(), nondet_int32(),
					     0, 0, 0, 0, fb_id, out, OUT_SIZE, nondet_uint8());

	} else {
		qc_esc_create_pwm_packet4_fb(nondet_int16(), nondet_int16(), nondet_int16(), nondet_int16(),
					     0, 0, 0, 0, fb_id, out, OUT_SIZE);
	}

	for (int k = 0; k < 4; k++) {
		int bit = decode_i16(out, k) & 1;
		assert(bit == (k == fb_id));                       /* P4 */
	}
}

int main(void)
{
#ifdef TEST_CHOICE
	int choice = TEST_CHOICE;                 /* rodar um teste por vez: -DTEST_CHOICE=n */
#else
	int choice = nondet_int();
	__VERIFIER_assume(choice >= 0 && choice <= 4);
#endif

	switch (choice) {
	case 0: test_parser_memory(); break;
	case 1: test_roundtrip();     break;
	case 2: test_rpm_command();   break;
	case 3: test_pwm_command();   break;
	case 4: test_feedback_bit();  break;
	}

	return 0;
}
