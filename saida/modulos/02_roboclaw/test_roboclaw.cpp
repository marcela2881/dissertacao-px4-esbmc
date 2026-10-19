/*
 * Harness de verificação — driver RoboClaw do PX4 (src/drivers/roboclaw)
 *
 * O código do driver NÃO é copiado: este arquivo inclui o Roboclaw.cpp ORIGINAL
 * do PX4 (commit fixado). Só as dependências externas (UART/POSIX, uORB,
 * parâmetros, ModuleBase, MixingOutput) são substituídas pelos stubs em
 * saida/stubs/ e pelas funções POSIX definidas abaixo.
 *
 * PRÉ-CONDIÇÕES (origem anotada):
 *   - RBCLW_ADDRESS: [128, 135]                       (module.yaml)
 *   - RBCLW_COUNTS_REV: >= 1                          (module.yaml, min: 1)
 *   - valores que o MixingOutput entrega ao driver (outputs[0..1]):
 *       min [0,127], max [127,254], disarmed 127, failsafe [0,257]
 *       -> faixa total [0, 257]                       (module.yaml, standard_params)
 *     No modo "termination" o mixer repassa o failsafe sem limitar
 *     (mixer_module.cpp, limitAndUpdateOutputs/actualFailsafeValue).
 *
 * PROPRIEDADES DE ESPECIFICAÇÃO (RoboClaw Series User Manual, seção
 * "Commands 0 - 7 Compatibility Commands": Rev. 6.0 (atual, firmware 4.3.0+), pp. 48-49;
 * mesmo texto na Rev. 5.5, pp. 48-49, e na Rev. 5, pp. 40-41):
 *   - RC1: comandos 0, 1, 4 e 5 (Drive Forward/Backwards M1/M2): o byte de
 *          dados vai de 0 a 127.
 *   - POSIX: write() devolve -1 em caso de erro (IEEE Std 1003.1, write()).
 *   RC1 conferida no manual em 19/10/2026: "Valid data range is 0 - 127".
 */

#include <cstdint>
#include <cmath>
#include <cassert>
#include <sys/types.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

extern "C" {
int      nondet_int();
uint8_t  nondet_uint8();
unsigned nondet_uint();
long     nondet_long();
float    nondet_float();
void     __VERIFIER_assume(int);   /* aceito pelo ESBMC e pelo CBMC */
}

/* --------------------------------------------------------------------------
 * Modelo da UART (POSIX). O dispositivo do outro lado é arbitrário: write()
 * pode enviar tudo, parte ou falhar (-1); read() devolve bytes quaisquer.
 * ------------------------------------------------------------------------ */
#define STUB_UART_MAX 16
uint8_t  g_last_write[STUB_UART_MAX];
size_t   g_last_write_len = 0;
bool     g_write_failed = false;      /* último write() devolveu -1 */
bool     g_check_write_error = false; /* R3 ativo */
uint8_t  g_drive_byte[2];             /* bytes de dados dos comandos 0/1/4/5 */
unsigned g_drive_n = 0;

extern "C" {
ssize_t write(int, const void *buf, size_t n)
{
	const uint8_t *b = static_cast<const uint8_t *>(buf);
	g_last_write_len = n;

	for (size_t i = 0; i < n && i < STUB_UART_MAX; i++) { g_last_write[i] = b[i]; }

	/* pacote de comando de acionamento: [endereço, comando, dado, CRC_H, CRC_L] */
	if (n == 5 && (b[1] == 0 || b[1] == 1 || b[1] == 4 || b[1] == 5) && g_drive_n < 2) {
		g_drive_byte[g_drive_n++] = b[2];
	}

	long r = nondet_long();
	__VERIFIER_assume(r == -1 || (r >= 0 && r <= (long)n));
	g_write_failed = (r == -1);
	return (ssize_t)r;
}

ssize_t read(int, void *buf, size_t n)
{
	long r = nondet_long();
	__VERIFIER_assume(r >= -1 && r <= (long)n);
	uint8_t *b = static_cast<uint8_t *>(buf);

	for (long i = 0; i < r; i++) { b[i] = nondet_uint8(); }

	return (ssize_t)r;
}

int select(int, fd_set *, fd_set *, fd_set *, struct timeval *)
{
	/* R3: depois de write() = -1 o driver não deveria esperar resposta */
	if (g_check_write_error) { assert(!g_write_failed); }

	return nondet_int();
}

int open(const char *, int, ...) { return nondet_int(); }
int close(int) { return 0; }
int tcgetattr(int, struct termios *) { return nondet_int(); }
int tcsetattr(int, int, const struct termios *) { return nondet_int(); }
int cfsetispeed(struct termios *, speed_t) { return nondet_int(); }
int cfsetospeed(struct termios *, speed_t) { return nondet_int(); }
}

#include "Roboclaw.hpp"
/* Inclui o Roboclaw.cpp ORIGINAL do PX4 (unidade de tradução única, como no PCA9685). */
#include "Roboclaw.cpp"

int32_t g_param_values[px4::params::PARAM_COUNT];

static void set_params()
{
	int32_t addr = nondet_int();
	__VERIFIER_assume(addr >= 128 && addr <= 135);           /* module.yaml */
	int32_t counts = nondet_int();
	__VERIFIER_assume(counts >= 1);                          /* module.yaml */
	g_param_values[px4::params::RBCLW_ADDRESS] = addr;
	g_param_values[px4::params::RBCLW_COUNTS_REV] = counts;
}

/* ---------------------------------------------------------------------------
 * R0 — readEncoder(): segurança de memória (buffers de resposta, CRC),
 * deslocamentos em swapBytesInt32 e divisão por RBCLW_COUNTS_REV.
 * Caminho de produção: Run() chama readEncoder() a cada ciclo.
 * Propriedades: automáticas do ESBMC.
 * ------------------------------------------------------------------------- */
void test_readEncoder_memory()
{
	set_params();
	Roboclaw dev("/dev/ttyS2", "RBCLW_BAUD");
	dev.readEncoder();
}

/* ---------------------------------------------------------------------------
 * R1 — RC1: byte de dados dos comandos de acionamento <= 127
 * Caminho de produção: MixingOutput -> updateOutputs() -> setMotorSpeed()
 * -> sendUnsigned7Bit().
 * ------------------------------------------------------------------------- */
void test_drive_byte_range()
{
	set_params();
	Roboclaw dev("/dev/ttyS2", "RBCLW_BAUD");

	float outputs[2];   /* Roboclaw::MAX_ACTUATORS = 2 (privado) */
	for (int i = 0; i < 2; i++) {
		outputs[i] = nondet_float();
		__VERIFIER_assume(outputs[i] >= 0.f && outputs[i] <= 257.f);   /* module.yaml */
	}

	dev.updateOutputs(outputs, 2, 1);

	for (unsigned i = 0; i < g_drive_n; i++) {
		assert(g_drive_byte[i] <= 127);                                /* RC1 */
	}
}

/* ---------------------------------------------------------------------------
 * R2 — Mesmo caminho de R1, só com a faixa normal do mixer (min..max,
 * disarmed): [0, 254]. Separa o caso normal do caso failsafe.
 * ------------------------------------------------------------------------- */
void test_drive_byte_range_normal()
{
	set_params();
	Roboclaw dev("/dev/ttyS2", "RBCLW_BAUD");

	float outputs[2];   /* Roboclaw::MAX_ACTUATORS = 2 (privado) */
	for (int i = 0; i < 2; i++) {
		outputs[i] = nondet_float();
		__VERIFIER_assume(outputs[i] >= 0.f && outputs[i] <= 254.f);   /* module.yaml */
	}

	dev.updateOutputs(outputs, 2, 1);

	for (unsigned i = 0; i < g_drive_n; i++) {
		assert(g_drive_byte[i] <= 127);                                /* RC1 */
	}
}

/* ---------------------------------------------------------------------------
 * R3 — Tratamento de erro de write(): se write() devolve -1 (POSIX), o driver
 * não deveria seguir para select()/read() esperando a resposta.
 * Caminho de produção: Run() -> readEncoder() -> receiveTransaction()
 * -> writeCommand(). A asserção está no stub de select().
 * ------------------------------------------------------------------------- */
void test_write_error_handling()
{
	set_params();
	Roboclaw dev("/dev/ttyS2", "RBCLW_BAUD");
	g_check_write_error = true;
	dev.readEncoder();
}

/* ---------------------------------------------------------------------------
 * R4 — setMotorDutyCycle() com qualquer float (sem pré-condição).
 * Método público, mas sem chamador no PX4 neste commit: robustez da API,
 * como o T4 do PCA9685. Rodar com --nan-check.
 * ------------------------------------------------------------------------- */
void test_duty_cycle_any()
{
	set_params();
	Roboclaw dev("/dev/ttyS2", "RBCLW_BAUD");
	dev.setMotorDutyCycle(Roboclaw::Motor::Right, nondet_float());
}

/* Ponto de entrada da verificação: --function harness_main.
 * O nome "main" fica ambíguo para o ESBMC porque o código original chama
 * ModuleBase::main() (roboclaw_main), que também existe no stub. */
int harness_main()
{
#ifdef TEST_CHOICE
	int choice = TEST_CHOICE;                 /* rodar um teste por vez: -DTEST_CHOICE=n */
#else
	int choice = nondet_int();
	__VERIFIER_assume(choice >= 0 && choice <= 4);
#endif

	switch (choice) {
	case 0: test_readEncoder_memory();      break;
	case 1: test_drive_byte_range();        break;
	case 2: test_drive_byte_range_normal(); break;
	case 3: test_write_error_handling();    break;
	case 4: test_duty_cycle_any();          break;
	}

	return 0;
}

int main() { return harness_main(); }
