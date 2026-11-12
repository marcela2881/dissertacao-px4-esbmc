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
    uint8_t num_events;
    Event events[EVENTS_ARRAY_SIZE];
};

int main() {
    arming_check_reply_s *reply = (arming_check_reply_s*)malloc(sizeof(arming_check_reply_s));
    memset(reply, 0, sizeof(arming_check_reply_s));

    // Componente externo envia num_events=200 (válido pra uint8, mas além do array[5])
    reply->num_events = 200;
    uint32_t modes = 0xFFFFFFFF;

    printf("num_events=%d EVENTS_ARRAY_SIZE=%d\n", reply->num_events, EVENTS_ARRAY_SIZE);
    printf("Iterando sobre reply->events[0..%d]...\n", reply->num_events - 1);

    // Loop real de externalChecks.cpp linhas 202-206
    for (int i = 0; i < reply->num_events; ++i) {
        memcpy(reply->events[i].arguments, &modes, sizeof(modes));
    }

    free(reply);
    return 0;
}
