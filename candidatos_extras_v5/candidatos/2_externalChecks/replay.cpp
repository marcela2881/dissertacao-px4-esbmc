/**
 * @file replay.cpp — candidato 2 (externalChecks.cpp)
 *
 * Replay do contraexemplo com AddressSanitizer.
 *
 * v5 (parecer B6): o valor de num_events NAO e mais escolhido a mao.
 * O rodar_tudo.sh extrai do trace do CBMC (reply.num_events=N) e passa
 * como argumento:  ./replay N
 * Sem argumento, usa 9 (valor do trace do CBMC 5.95.1 em 12/10/2026).
 *
 * v5 (parecer C11): a struct agora tem os mesmos campos do harness
 * (timestamp e os cinco campos de status estavam omitidos no v4).
 * v5 (parecer C7): sem free() depois da corrupcao.
 *
 * O objeto e alocado no heap e tem exatamente sizeof(arming_check_reply_s),
 * entao a escrita em events[5] passa do fim da alocacao e o ASan a ve.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define EVENTS_ARRAY_SIZE 5

struct Event {
	uint32_t id;
	uint8_t log_levels;
	uint8_t arguments[4];
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
	uint8_t num_events;
	Event events[EVENTS_ARRAY_SIZE];
};

int main(int argc, char **argv)
{
	int n = (argc > 1) ? atoi(argv[1]) : 9;

	arming_check_reply_s *reply = (arming_check_reply_s *)malloc(sizeof(arming_check_reply_s));
	memset(reply, 0, sizeof(arming_check_reply_s));

	reply->num_events = (uint8_t)n;
	uint32_t modes = 0xFFFFFFFF;

	printf("num_events=%d (do contraexemplo) EVENTS_ARRAY_SIZE=%d sizeof(reply)=%zu\n",
	       reply->num_events, EVENTS_ARRAY_SIZE, sizeof(arming_check_reply_s));

	// Laco real de externalChecks.cpp (8be2590c15)
	for (int i = 0; i < reply->num_events; ++i) {
		memcpy(reply->events[i].arguments, &modes, sizeof(modes));
	}

	printf("Se chegou aqui: nenhuma deteccao\n");
	return 0;
}
