/*
 * REPLAY (teste de reprodução) — RoboClaw, contraexemplos R1 e R3
 *
 * Compila o Roboclaw.cpp ORIGINAL do PX4 com o g++ comum e EXECUTA o driver
 * com os valores dos contraexemplos do ESBMC, mostrando o que ele realmente
 * enviaria pela UART. O hardware (UART) é simulado de forma determinística:
 *   - write(): grava o pacote enviado e devolve o valor escolhido pelo teste;
 *   - read()/select(): o "RoboClaw" responde ACK (0xFF).
 * Parâmetros: RBCLW_ADDRESS = 128 (0x80, padrão), RBCLW_COUNTS_REV = 1200 (padrão).
 */
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <sys/types.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

/* ---- UART simulada ---- */
static uint8_t g_pkts[8][16];
static size_t  g_pkt_len[8];
static int     g_npkts = 0;
static long    g_write_ret = 0;        /* 0 = devolver n (sucesso); -1 = simular erro */
static int     g_select_calls = 0;

extern "C" {
ssize_t write(int, const void *buf, size_t n)
{
	if (g_npkts < 8) {
		memcpy(g_pkts[g_npkts], buf, n < 16 ? n : 16);
		g_pkt_len[g_npkts++] = n;
	}
	return g_write_ret == -1 ? -1 : (ssize_t)n;
}
ssize_t read(int, void *buf, size_t n)
{
	if (n >= 1) { ((uint8_t *)buf)[0] = 0xFF; return 1; }   /* ACK */
	return 0;
}
int select(int, fd_set *, fd_set *, fd_set *, struct timeval *) { g_select_calls++; return 1; }
int open(const char *, int, ...) { return 3; }
int close(int) { return 0; }
int tcgetattr(int, struct termios *) { return 0; }
int tcsetattr(int, int, const struct termios *) { return 0; }
int cfsetispeed(struct termios *, speed_t) { return 0; }
int cfsetospeed(struct termios *, speed_t) { return 0; }
}

#include "Roboclaw.hpp"
#include "Roboclaw.cpp"     /* arquivo ORIGINAL do PX4 */

int32_t g_param_values[px4::params::PARAM_COUNT];

static const char *nome_cmd(uint8_t c)
{
	switch (c) {
	case 0: return "DriveForwardMotor1";
	case 1: return "DriveBackwardsMotor1";
	case 4: return "DriveForwardMotor2";
	case 5: return "DriveBackwardsMotor2";
	default: return "?";
	}
}

int main()
{
	g_param_values[px4::params::RBCLW_ADDRESS] = 128;
	g_param_values[px4::params::RBCLW_COUNTS_REV] = 1200;
	int violacoes = 0;

	printf("=== R1: byte de dados dos comandos de acionamento (faixa esperada 0..127) ===\n");
	printf("%-10s %-22s %-6s %s\n", "outputs[0]", "comando enviado", "byte", "situação");
	const float valores[] = {0.f, 127.f, 200.f, 254.f, 254.5f, 255.f, 256.f, 257.f};

	for (float v : valores) {
		Roboclaw dev("/dev/ttyS2", "RBCLW_BAUD");
		g_npkts = 0; g_write_ret = 0;
		float outputs[2] = {v, 127.f};
		dev.updateOutputs(outputs, 2, 1);
		/* 1º pacote = motor direito: [endereço, comando, dado, CRC_H, CRC_L] */
		uint8_t cmd = g_pkts[0][1], dado = g_pkts[0][2];
		bool viola = dado > 127;
		violacoes += viola;
		printf("%-10.1f %-22s %-6u %s\n", v, nome_cmd(cmd), dado,
		       viola ? "VIOLA (> 127)" : (v > 254.f ? "ok" : "ok (faixa normal do mixer)"));
	}

	printf("\n=== R3: write() devolve -1 (erro POSIX) durante readEncoder() ===\n");
	{
		Roboclaw dev("/dev/ttyS2", "RBCLW_BAUD");
		g_npkts = 0; g_write_ret = -1; g_select_calls = 0;
		int r = dev.readEncoder();
		printf("write() devolveu -1 em todas as escritas\n");
		printf("select() chamado %d vez(es) depois da falha (esperado: 0)\n", g_select_calls);
		printf("readEncoder() retornou %d (%s)\n", r, r == OK ? "OK" : "ERROR");
		if (g_select_calls > 0) {
			printf("=> o driver tratou a falha de escrita como SUCESSO e foi esperar resposta (R3 reproduzido)\n");
			violacoes++;
		}
	}

	printf("\nRESUMO REPLAY RoboClaw: %d situação(ões) reproduzida(s)\n", violacoes);
	return 0;
}
