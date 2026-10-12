/**
 * @file harness.cpp — candidato 2 (externalChecks.cpp)
 *
 * MODULO PX4: src/modules/commander/HealthAndArmingChecks/checks/externalChecks.cpp
 *             + msg/versioned/ArmingCheckReply.msg
 * FUNCAO:     laco sobre reply.num_events indexando reply.events[i]
 * BASE:       PX4 commit 8be2590c15 (04/06/2026)
 *
 * ArmingCheckReply.msg define
 *     uint8 num_events            // 0..255
 *     Event[5] events             // array fixo de 5 posicoes
 * e no 8be2590c15 nenhuma checagem "num_events <= 5" existe antes do laco.
 * reply chega por uORB de um componente externo (modo ROS 2 registrado).
 *
 * SITUACAO UPSTREAM: corrigido no PX4 pelo commit ca1d36476a (PR #28598,
 * 14/09/2026), que limita num_events na ingestao. Nao abrir issue.
 * Compilar com -DCORRECAO_UPSTREAM aplica essa correcao ao harness
 * (etapa vii: reverificacao no patch).
 *
 * v5 (parecer B18): a assercao deixou de ser sobre a ENTRADA
 * (num_events <= 5, que falha para qualquer entrada > 5 mesmo com laco
 * protegido) e passou a ser sobre o INDICE efetivamente acessado.
 *
 * Estado inicial: num_events e livre (nondet), nao assumido.
 */

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

extern uint8_t nondet_uint8();

// ================== ESTRUTURA REAL, extraida de msg/versioned/ArmingCheckReply.msg ==================
#define EVENTS_ARRAY_SIZE 5  // "Event[5] events" no .msg

struct Event {
	uint32_t id;
	uint8_t log_levels;
	uint8_t arguments[4]; // reduzido de 16 para 4 (= sizeof(modes)) apenas para
	                       // aliviar a codificacao SMT; nao afeta a propriedade
	                       // testada (o indice i ainda pode passar de 5).
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
	uint8_t num_events;          // uint8: 0..255, SEM relacao com EVENTS_ARRAY_SIZE
	Event events[EVENTS_ARRAY_SIZE]; // array fixo de 5
};

// ================== CORRECAO UPSTREAM (ca1d36476a), aplicada na ingestao ==================
static void ingest_reply(arming_check_reply_s *reply)
{
#ifdef CORRECAO_UPSTREAM
	static const uint8_t max_num_events = sizeof(reply->events) / sizeof(reply->events[0]);

	if (reply->num_events > max_num_events) {
		reply->num_events = max_num_events;
	}
#else
	(void)reply;
#endif
}

// ================== LOGICA REAL EXTRAIDA de externalChecks.cpp (8be2590c15) ==================
//
//   for (int i = 0; i < reply.num_events; ++i) {
//       // set the modes, which is the first argument
//       memcpy(reply.events[i].arguments, &modes, sizeof(modes));
//       reporter.addExternalEvent(reply.events[i], modes);
//   }
//
// "modes" e um NavModes (uint32_t) no codigo real; simplificado aqui.

void process_external_reply(arming_check_reply_s *reply, uint32_t modes)
{
	for (int i = 0; i < reply->num_events; ++i) {
		// PROPRIEDADE (B18): o indice acessado tem de ser valido.
		assert(i < EVENTS_ARRAY_SIZE);
		memcpy(reply->events[i].arguments, &modes, sizeof(modes));
		// reporter.addExternalEvent simplificado: apenas toca o struct
		(void)reply->events[i].id;
	}
}

int main()
{
	arming_check_reply_s reply;
	memset(&reply, 0, sizeof(reply));

	// num_events vem de uma mensagem uORB publicada por um componente
	// externo: dado nao confiavel, livre de 0 a 255.
	reply.num_events = nondet_uint8();

	uint32_t modes = nondet_uint8();

	ingest_reply(&reply);
	process_external_reply(&reply, modes);

	return 0;
}
