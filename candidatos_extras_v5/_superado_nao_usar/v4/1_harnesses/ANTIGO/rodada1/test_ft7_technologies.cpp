/**
 * @file test_ft7_technologies.cpp
 *
 * OBJETIVO: Teste de código REAL extraído de
 * src/drivers/wind_sensor/ft_technologies/ft7_technologies.cpp e .hpp
 * FUNÇÃO TESTADA: Ft7Technologies::collect() - acumulação de
 * _byte_counter e _msg_byte_counter sem checagem de limite.
 * MÉTODO: Bounded Model Checking com ESBMC / CBMC
 *
 * Candidato #3 (confiança MUITO forte): _readbuf[30] é indexado por
 * _byte_counter (int), que persiste entre chamadas de collect() e
 * SÓ é resetado quando um '\n' é encontrado na linha. Cada chamada
 * de collect() pode ler até 19 bytes (sizeof(_linebuf)-1, com
 * _linebuf[20]). Duas chamadas sem '\n' (19+19=38 > 30) já estouram
 * o buffer. O mesmo padrão se repete em _raw_speed[5], _raw_angle[5],
 * _raw_status[2], _raw_checksum[3], indexados por _msg_byte_counter
 * sem nenhuma checagem visível.
 */

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

extern char nondet_char();
extern int nondet_int();

// ================== TAMANHOS REAIS, extraídos de ft7_technologies.hpp ==================
#define LINEBUF_SIZE 20   // char _linebuf[20] {};   (hpp linha 83)
#define READBUF_SIZE 30   // char _readbuf[30] {};   (hpp linha 79)
#define RAW_SPEED_SIZE 5
#define RAW_ANGLE_SIZE 5
#define RAW_STATUS_SIZE 2
#define RAW_CHECKSUM_SIZE 3

// ================== ESTADO REAL (membros da classe Ft7Technologies) ==================
char _readbuf[READBUF_SIZE]{};
char _raw_angle[RAW_ANGLE_SIZE];
char _raw_speed[RAW_SPEED_SIZE];
char _raw_status[RAW_STATUS_SIZE];
char _raw_checksum[RAW_CHECKSUM_SIZE];

int _byte_counter{0};
int _msg_byte_counter{0};
int _msg_part_counter{0};
int _checksum_counter{0};

// ================== LÓGICA REAL EXTRAÍDA de collect(), linhas 158-234 ==================
// (laço principal e o corpo dentro de "for (int i = 0; i < ret; i++)")
//
// Simplificação para o harness: cada chamada simula UMA leitura de
// 'ret' bytes (nondet, limitado a sizeof(readbuf)-1 = 19, exatamente
// como no código real: "unsigned readlen = sizeof(readbuf) - 1;").
// O conteúdo de cada byte também é nondet.

void simulate_collect_call(int ret, const char *readbuf)
{
	for (int i = 0; i < ret; i++) {

		// linha 161-162 do arquivo real - SEM checagem de limite
		_readbuf[_byte_counter] = readbuf[i];
		_byte_counter += 1;

		if (readbuf[i] == '\n') {
			// reset de contadores (linhas 191-201 do arquivo real)
			_msg_part_counter = 0;
			_byte_counter = 0;
			_msg_byte_counter = 0;
			_checksum_counter = 0;

		} else if (readbuf[i] == '$' || readbuf[i] == ',' || readbuf[i] == '=' ||
			   readbuf[i] == '*' || readbuf[i] == '\r') {
			_msg_part_counter += 1;
			_msg_byte_counter = 0;

		} else {
			if (readbuf[i] != '.') {
				// linhas 213-226 do arquivo real - SEM checagem de limite
				if (_msg_part_counter == 3) {
					_raw_speed[_msg_byte_counter] = readbuf[i];

				} else if (_msg_part_counter == 4) {
					_raw_angle[_msg_byte_counter] = readbuf[i];

				} else if (_msg_part_counter == 5) {
					_raw_status[_msg_byte_counter] = readbuf[i];

				} else if (_msg_part_counter == 6) {
					_checksum_counter += 1;
					_raw_checksum[_msg_byte_counter] = readbuf[i];
				}

				_msg_byte_counter += 1;
			}
		}
	}
}

int main()
{
	char readbuf[LINEBUF_SIZE - 1]; // 19 bytes, igual ao código real

	for (int i = 0; i < LINEBUF_SIZE - 1; i++) {
		readbuf[i] = nondet_char();
		// Excluímos deliberadamente '\n' para simular o cenário do
		// bug: uma linha mal formada / ruído de UART que nunca
		// termina, forçando o contador a crescer entre chamadas.
		__VERIFIER_assume(readbuf[i] != '\n');
	}

	int ret1 = nondet_int();
	__VERIFIER_assume(ret1 >= 0 && ret1 <= LINEBUF_SIZE - 1);
	simulate_collect_call(ret1, readbuf);

	int ret2 = nondet_int();
	__VERIFIER_assume(ret2 >= 0 && ret2 <= LINEBUF_SIZE - 1);
	simulate_collect_call(ret2, readbuf);

	// PROPRIEDADE: _byte_counter nunca deveria exceder o tamanho real do buffer.
	assert(_byte_counter <= READBUF_SIZE);

	return 0;
}
