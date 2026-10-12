/**
 * @file test_logger_write_info_multiple.cpp
 *
 * OBJETIVO: Teste de código REAL extraído de
 * src/modules/logger/logger.cpp (Logger::write_info_multiple) e
 * src/modules/logger/messages.h (struct ulog_message_info_multiple_s)
 * MÉTODO: Bounded Model Checking com ESBMC / CBMC
 *
 * Candidato #4 (defeito LATENTE, não confirmado explorável com as
 * chamadas reais de hoje): msg.key_len é uint8_t (0..255), mas recebe
 * o retorno de snprintf() sem checagem. Em todos os call sites reais
 * (logger.cpp linhas 1629, 1672, 1742), "name" é uma string curta
 * fixa pelo desenvolvedor ("boot_console_output", nomes de perf
 * counter, etc.), nunca perto de 255 caracteres. Este harness trata
 * "name" como dado simbólico (nondet), sem assumir as chamadas reais,
 * para testar formalmente se a fraqueza estrutural é alcançável em
 * teoria - é exatamente o tipo de questão que análise manual não
 * resolve com confiança.
 */

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

extern char nondet_char();
extern size_t nondet_size_t();

#define ULOG_MSG_HEADER_LEN 3 // valor ilustrativo consistente com o uso real (msg_size - ULOG_MSG_HEADER_LEN)
#define KEY_VALUE_STR_SIZE 1200 // messages.h linha 209: char key_value_str[1200]

// ================== ESTRUTURA REAL, extraída de messages.h linhas 203-210 ==================
struct ulog_message_info_multiple_s {
	uint16_t msg_size;
	uint8_t msg_type;
	uint8_t is_continued;
	uint8_t key_len;                       // <-- uint8_t, o tipo estreito do defeito
	char key_value_str[KEY_VALUE_STR_SIZE]; // 1200 bytes
};

// ================== LÓGICA REAL EXTRAÍDA de Logger::write_info_multiple(), linhas 1967-1991 ==================
//
//   void Logger::write_info_multiple(LogType type, const char *name, const char *value, bool is_continued)
//   {
//       ulog_message_info_multiple_s msg;
//       uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);
//       ...
//       size_t vlen = strlen(value);
//       msg.key_len = snprintf(msg.key_value_str, sizeof(msg.key_value_str), "char[%zu] %s", vlen, name);
//       size_t msg_size = sizeof(msg) - sizeof(msg.key_value_str) + msg.key_len;
//
//       if (vlen < (sizeof(msg) - msg_size)) {
//           memcpy(&buffer[msg_size], value, vlen + 1);
//           msg_size += vlen;
//           msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;
//           write_message(type, buffer, msg_size);
//       } else {
//           PX4_ERR("info_multiple str too long (%" PRIu8 "), key=%s", msg.key_len, msg.key_value_str);
//       }
//   }

void write_info_multiple(const char *name, const char *value)
{
	ulog_message_info_multiple_s msg;
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);

	size_t vlen = strlen(value);

	// linha 1976 do arquivo real: retorno de snprintf atribuído a uint8_t
	msg.key_len = snprintf(msg.key_value_str, sizeof(msg.key_value_str), "char[%zu] %s", vlen, name);

	size_t msg_size = sizeof(msg) - sizeof(msg.key_value_str) + msg.key_len;

	if (vlen < (sizeof(msg) - msg_size)) {
		memcpy(&buffer[msg_size], value, vlen + 1);
		msg_size += vlen;
	}
}

int main()
{
	// "name" tratado como simbólico para testar a fraqueza estrutural
	// independentemente das chamadas reais de hoje (todas usam strings
	// curtas fixas). Limitamos o comprimento simbólico a um valor que
	// ainda cabe no buffer real (1200) mas já excede 255 (limite do
	// uint8_t key_len), para focar exatamente no cenário de truncamento.
	char name[300];

	for (int i = 0; i < 299; i++) {
		name[i] = nondet_char();
		__VERIFIER_assume(name[i] != '\0'); // string sem terminador embutido no meio
	}
	name[299] = '\0';

	char value[8];

	for (int i = 0; i < 7; i++) {
		value[i] = nondet_char();
		__VERIFIER_assume(value[i] != '\0');
	}
	value[7] = '\0';

	write_info_multiple(name, value);

	return 0;
}
