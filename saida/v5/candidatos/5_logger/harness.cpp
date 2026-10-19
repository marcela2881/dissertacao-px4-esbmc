/**
 * @file harness.cpp — candidato 5 (logger.cpp), SEM VIOLACAO DE MEMORIA
 *
 * MODULO PX4: src/modules/logger/logger.cpp, Logger::write_info_multiple()
 *             (linhas 1967-1991 no 8be2590c15) e messages.h (linhas 203-210)
 * BASE: PX4 commit 8be2590c15; a funcao e igual no main de 12/10/2026.
 *
 * Candidato #5 (v5: numeracao corrigida, parecer B22). Defeito LATENTE:
 * msg.key_len e uint8_t (0..255) e recebe o retorno de snprintf() sem
 * checagem. Todos os call sites reais passam nomes curtos e fixos.
 *
 * O QUE ESTE HARNESS PROVA E O QUE NAO PROVA (v5, pareceres A6, A7, B15):
 *
 * - O modelo de snprintf do ESBMC devolve um valor nao-deterministico
 *   irrestrito quando o argumento %s e construido em tempo de execucao
 *   (ver sondagem_snprintf.c). Logo, no ESBMC, key_len cobre os 256 valores
 *   possiveis, e o SUCCESSFUL significa: nenhuma violacao de limites de
 *   memoria para QUALQUER valor de key_len em 0..255. Nao significa que o
 *   truncamento foi exercitado, e o k = 300 do log e so o comprimento do
 *   laco concreto de 299 iteracoes do main(), nao profundidade de prova.
 *
 * - Para testar o truncamento em si, compilar com -DMODELO_KEY_LEN_NONDET:
 *   o retorno de snprintf vira um int livre em 0..1199 (a faixa que o
 *   snprintf real pode devolver com este buffer e este formato), atribuido
 *   ao uint8_t como no codigo real. Uma assercao explicita
 *   (ret_snprintf <= 255) mostra se o truncamento e alcancavel, e as
 *   checagens de limites mostram se ele leva a acesso fora de limites.
 *   Rodar com --multi-property para ver as duas coisas no mesmo log.
 *   (O --conversion-check do CBMC faria o mesmo, mas estoura a memoria
 *   neste harness; por isso a assercao explicita.)
 *
 * - Assercao explicita (B15): msg_size nunca passa de sizeof(msg).
 *
 * v5 (C8): ULOG_MSG_HEADER_LEN, que nao era usado, foi removido.
 */

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

extern char nondet_char();
extern int nondet_int();

#define KEY_VALUE_STR_SIZE 1200 // messages.h linha 209: char key_value_str[1200]

// ================== ESTRUTURA REAL, extraida de messages.h linhas 203-210 ==================
struct ulog_message_info_multiple_s {
	uint16_t msg_size;
	uint8_t msg_type;
	uint8_t is_continued;
	uint8_t key_len;                       // <-- uint8_t, o tipo estreito
	char key_value_str[KEY_VALUE_STR_SIZE]; // 1200 bytes
};

// ================== LOGICA REAL, Logger::write_info_multiple() (8be2590c15, linhas 1976-1983) ==================
void write_info_multiple(const char *name, const char *value)
{
	ulog_message_info_multiple_s msg;
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);

	size_t vlen = strlen(value);

#ifdef MODELO_KEY_LEN_NONDET
	(void)name;
	int ret_snprintf = nondet_int();
	__VERIFIER_assume(ret_snprintf >= 0 && ret_snprintf < KEY_VALUE_STR_SIZE);
	// PROPRIEDADE DO TRUNCAMENTO: o valor cabe em uint8_t? (falha = truncamento alcancavel)
	assert(ret_snprintf <= 255);
	msg.key_len = ret_snprintf;   // mesma conversao implicita int -> uint8_t da linha 1977
#else
	// linha 1977: retorno de snprintf atribuido a uint8_t
	msg.key_len = snprintf(msg.key_value_str, sizeof(msg.key_value_str), "char[%zu] %s", vlen, name);
#endif

	size_t msg_size = sizeof(msg) - sizeof(msg.key_value_str) + msg.key_len;

	// PROPRIEDADE explicita (B15)
	assert(msg_size <= sizeof(msg));

	if (vlen < (sizeof(msg) - msg_size)) {
		memcpy(&buffer[msg_size], value, vlen + 1);
		msg_size += vlen;
		assert(msg_size < sizeof(msg));
	}
}

int main()
{
	// "name" simbolico, com comprimento 299 (cabe em 1200 mas passa de 255).
	char name[300];
#ifndef MODELO_KEY_LEN_NONDET
	for (int i = 0; i < 299; i++) {
		name[i] = nondet_char();
		__VERIFIER_assume(name[i] != '\0');
	}
#endif
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
