/*
 * Harness de verificação — protocolo serial do tap_esc do PX4
 * (src/drivers/tap_esc: tap_esc_common.cpp e drv_tap_esc.h)
 *
 * O código NÃO é copiado: o harness inclui o tap_esc_common.cpp ORIGINAL do
 * PX4 (commit fixado). Stubs: <fcntl.h> (ver saida/stubs/fcntl.h), log,
 * board_config e as funções POSIX read/write definidas abaixo. Recorte: a
 * biblioteca do protocolo (montagem, envio, recepção e leitura de pacotes);
 * a classe TAP_ESC em si não entra neste harness.
 *
 * PRÉ-CONDIÇÕES (origem anotada):
 *   - estado do buffer circular ESC_UART_BUF: head < 128, tail < 128 e
 *     dat_cnt < 128 (UART_BUFFER_SIZE; mantido por read_data_from_uart,
 *     ver T1); o driver começa com o buffer zerado (TAP_ESC.hpp).
 *   - len dos pacotes enviados < sizeof(d) = 100: os remetentes usam
 *     sizeof(InfoRequest)=2, sizeof(EscbusTunePacket)=5, 2*canais <= 16
 *     (TAP_ESC.cpp).
 *
 * PROPRIEDADES (do próprio protocolo em drv_tap_esc.h / tap_esc_common.cpp):
 *   - TP1: o buffer circular nunca é acessado fora dos limites e o invariante
 *          (head, tail, dat_cnt < 128) se mantém após cada leitura da UART.
 *   - TP2: um pacote enviado por send_packet é aceito por
 *          parse_tap_esc_feedback (formato e CRC8 consistentes) com os mesmos
 *          len, msg_id e dados.
 */

#include <cstdint>
#include <cstring>
#include <cassert>
#include <unistd.h>

extern "C" {
int      nondet_int();
long     nondet_long();
uint8_t  nondet_uint8();
void     __VERIFIER_assume(int);   /* aceito pelo ESBMC e pelo CBMC */
}

/* --------------------------------------------------------------------------
 * Modelo da UART (POSIX): write() guarda os bytes enviados; read() devolve
 * de -1 a n bytes quaisquer.
 * ------------------------------------------------------------------------ */
#define STUB_TX_MAX 128
uint8_t g_tx[STUB_TX_MAX];
size_t  g_tx_len = 0;

extern "C" {
ssize_t write(int, const void *buf, size_t n)
{
	const uint8_t *b = static_cast<const uint8_t *>(buf);
	g_tx_len = n;

	for (size_t i = 0; i < n && i < STUB_TX_MAX; i++) { g_tx[i] = b[i]; }

	return (ssize_t)n;
}

ssize_t read(int, void *buf, size_t n)
{
	long r = nondet_long();
	__VERIFIER_assume(r >= -1 && r <= (long)n);
	uint8_t *b = static_cast<uint8_t *>(buf);

	for (long i = 0; i < r; i++) { b[i] = nondet_uint8(); }

	return (ssize_t)r;
}
}

/* arquivo ORIGINAL do PX4 (unidade de tradução única) */
#include "tap_esc_common.cpp"

using namespace tap_esc_common;

#ifndef N_BYTES
#define N_BYTES 16      /* bytes no buffer em T0 (24 deu Timed out em 300 s, 18/10/2026) */
#endif

static int invariante(const ESC_UART_BUF *b)
{
	return b->head < UART_BUFFER_SIZE && b->tail < UART_BUFFER_SIZE && b->dat_cnt < UART_BUFFER_SIZE;
}

/* ---------------------------------------------------------------------------
 * T0 — TP1 no leitor: parse_tap_esc_feedback com até N_BYTES bytes quaisquer
 * em qualquer posição do buffer circular (head livre): memória de
 * esc_feedback_buf[] e de packetdata->d.bytes[]. O estado da máquina do
 * leitor é static (não pode ser escolhido pelo harness), por isso o teste
 * percorre sequências de bytes a partir do estado inicial HEAD.
 * ------------------------------------------------------------------------- */
void test_parser_memory()
{
	ESC_UART_BUF buf;

	for (int i = 0; i < UART_BUFFER_SIZE; i++) { buf.esc_feedback_buf[i] = nondet_uint8(); }

	buf.head = nondet_uint8();
	buf.dat_cnt = nondet_uint8();
	__VERIFIER_assume(buf.head < UART_BUFFER_SIZE && buf.dat_cnt <= N_BYTES);
	buf.tail = (buf.head + buf.dat_cnt) % UART_BUFFER_SIZE;

	EscPacket pkt;
	parse_tap_esc_feedback(&buf, &pkt);
	assert(invariante(&buf));
}

/* ---------------------------------------------------------------------------
 * T1 — TP1 na recepção: read_data_from_uart a partir de QUALQUER estado que
 * satisfaz o invariante; a UART devolve de -1 a 128 bytes quaisquer.
 * Se o invariante se mantém, vale para qualquer número de leituras (indução).
 * ------------------------------------------------------------------------- */
void test_uart_buffer()
{
	ESC_UART_BUF buf;
	buf.head = nondet_uint8();
	buf.tail = nondet_uint8();
	buf.dat_cnt = nondet_uint8();
	__VERIFIER_assume(invariante(&buf));

	read_data_from_uart(0, &buf);
	assert(invariante(&buf));
}

/* ---------------------------------------------------------------------------
 * T2/T4 — TP2: ida e volta (send_packet -> parse_tap_esc_feedback) com um
 * pacote de tamanho real do driver e dados livres:
 *   T2: InfoRequest (len 2)  |  T4: EscbusTunePacket (len 5)
 * ------------------------------------------------------------------------- */
static void roundtrip(uint8_t len, uint8_t msg_id)
{
	EscPacket tx{};
	tx.head = PACKET_HEAD;
	tx.len = len;
	tx.msg_id = msg_id;

	for (int i = 0; i < len; i++) { tx.d.bytes[i] = nondet_uint8(); }

	int n = send_packet(0, tx, -1);
	assert(n == len + 4 && g_tx_len == (size_t)n);

	/* cabeçalho enviado confere com o pacote (formato do protocolo) */
	assert(g_tx[0] == PACKET_HEAD && g_tx[1] == len && g_tx[2] == msg_id);

	/* Bytes recebidos = bytes enviados. head/len/msg_id são gravados com os
	 * valores (já conferidos acima) para que o ESBMC saiba que o tamanho é fixo;
	 * senão ele desenrola o laço do CRC até o limite global (Timed out na 1ª
	 * versão, 18/10/2026). Dados e CRC vêm do que foi enviado. */
	ESC_UART_BUF buf{};
	buf.esc_feedback_buf[0] = PACKET_HEAD;
	buf.esc_feedback_buf[1] = len;
	buf.esc_feedback_buf[2] = msg_id;

	for (int i = 3; i < n; i++) { buf.esc_feedback_buf[i] = g_tx[i]; }

	buf.dat_cnt = n;
	buf.tail = n;

	EscPacket rx{};
	int ret = parse_tap_esc_feedback(&buf, &rx);

	assert(ret == 0);                                     /* TP2: aceito */
	assert(rx.len == len && rx.msg_id == msg_id);         /* TP2: mesmos campos */

	for (int i = 0; i < len; i++) { assert(rx.d.bytes[i] == tx.d.bytes[i]); }
}

/* ---------------------------------------------------------------------------
 * T3 — send_packet com qualquer len aceito (< 100) e qualquer msg_id:
 * memória (CRC gravado em d.bytes[len], leitura de len+4 bytes do pacote).
 * ------------------------------------------------------------------------- */
void test_send_any_len()
{
	EscPacket tx;
	tx.head = PACKET_HEAD;
	tx.len = nondet_uint8();
	tx.msg_id = nondet_uint8();
	__VERIFIER_assume(tx.len < sizeof(tx.d));

	int n = send_packet(0, tx, -1);
	assert(n == tx.len + 4);
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
	case 0: test_parser_memory(); break;
	case 1: test_uart_buffer();   break;
	case 2: roundtrip(sizeof(InfoRequest), ESCBUS_MSG_ID_REQUEST_INFO); break;
	case 3: test_send_any_len();  break;
	case 4: roundtrip(sizeof(EscbusTunePacket), ESCBUS_MSG_ID_TUNE); break;
	}

	return 0;
}

int main() { return harness_main(); }
