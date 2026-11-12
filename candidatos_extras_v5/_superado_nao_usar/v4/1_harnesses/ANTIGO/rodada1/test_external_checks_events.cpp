/**
 * @file test_external_checks_events.cpp
 *
 * OBJETIVO: Teste de código REAL extraído de
 * src/modules/commander/HealthAndArmingChecks/checks/externalChecks.cpp
 * e da definição de mensagem uORB msg/versioned/ArmingCheckReply.msg
 * FUNÇÃO TESTADA: loop sobre reply.num_events indexando reply.events[i]
 * MÉTODO: Bounded Model Checking com ESBMC / CBMC
 *
 * Candidato #2 (confiança forte): ArmingCheckReply.msg define:
 *     uint8 num_events            // 0..255
 *     Event[5] events             // array FIXO de 5 posições
 * Nenhuma checagem de "num_events <= 5" foi encontrada em
 * externalChecks.cpp antes do loop (confirmado via grep no arquivo
 * completo). reply vem de um componente externo via uORB (ex.: modo
 * ROS 2 registrado), portanto é dado não confiável, na mesma classe
 * de risco do driver GPS.
 */

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

extern uint8_t nondet_uint8();

// ================== ESTRUTURA REAL, extraída de msg/versioned/ArmingCheckReply.msg ==================
#define EVENTS_ARRAY_SIZE 5  // "Event[5] events" no .msg

struct Event {
	uint32_t id;
	uint8_t log_levels;
	uint8_t arguments[4]; // reduzido de 16 para 4 (= sizeof(modes)) apenas para
	                       // aliviar a codificação SMT; não afeta a propriedade
	                       // testada (o índice i ainda pode passar de 5).
};

struct arming_check_reply_s {
	uint64_t timestamp;
	uint8_t request_id;
	uint8_t registration_id;
	uint8_t health_component_index;
	bool health_component_is_present;
	bool health_component_warning;
	bool health_component_error;
	bool can_arm_and_run;
	uint8_t num_events;          // uint8: 0..255, SEM relação com EVENTS_ARRAY_SIZE
	Event events[EVENTS_ARRAY_SIZE]; // array fixo de 5
};

// ================== LÓGICA REAL EXTRAÍDA de externalChecks.cpp linhas 202-206 ==================
//
//   for (int i = 0; i < reply.num_events; ++i) {
//       // set the modes, which is the first argument
//       memcpy(reply.events[i].arguments, &modes, sizeof(modes));
//       reporter.addExternalEvent(reply.events[i], modes);
//   }
//
// "modes" é um NavModes (uint32_t) no código real; simplificado aqui.

void process_external_reply(arming_check_reply_s *reply, uint32_t modes)
{
	for (int i = 0; i < reply->num_events; ++i) {
		// linha 204 do arquivo real
		memcpy(reply->events[i].arguments, &modes, sizeof(modes));
		// linha 206 (reporter.addExternalEvent) simplificada: apenas
		// tocamos o struct para expor o mesmo acesso fora dos limites
		(void)reply->events[i].id;
	}
}

int main()
{
	arming_check_reply_s reply;
	memset(&reply, 0, sizeof(reply));

	// num_events vem de uma mensagem uORB publicada por um componente
	// externo: dado não confiável, livre de 0 a 255 (range completo do uint8).
	reply.num_events = nondet_uint8();

	uint32_t modes = nondet_uint8();

	process_external_reply(&reply, modes);

	// PROPRIEDADE: nunca deveríamos acessar além do array real de 5 posições.
	assert(reply.num_events <= EVENTS_ARRAY_SIZE);

	return 0;
}
