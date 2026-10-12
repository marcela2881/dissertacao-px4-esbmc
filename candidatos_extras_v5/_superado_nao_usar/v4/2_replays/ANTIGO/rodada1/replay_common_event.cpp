#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define EVENT_BUFFER_HEADER_SIZE 6
#define EVENT_BUFFER_SIZE 72

int main() {
    uint8_t *_event_buffer = (uint8_t*)malloc(EVENT_BUFFER_SIZE);
    memset(_event_buffer, 0, EVENT_BUFFER_SIZE);

    // _next_buffer_idx já perto do limite — contraexemplo do ESBMC: idx=74
    int _next_buffer_idx = 74;
    unsigned args_size = 10;
    unsigned total_size = EVENT_BUFFER_HEADER_SIZE + args_size; // = 16

    printf("EVENT_BUFFER_SIZE=%d _next_buffer_idx=%d total_size=%d\n",
           EVENT_BUFFER_SIZE, _next_buffer_idx, total_size);

    // Guarda real de Common.hpp — mistura sinalizado/não-sinalizado:
    // sizeof (size_t) - int = conversão pra unsigned -> underflow se idx > buffer
    if (total_size > (size_t)EVENT_BUFFER_SIZE - _next_buffer_idx) {
        printf("Guarda deveria disparar aqui, mas pode falhar com underflow\n");
    }

    // Escrita real: memset além do buffer
    printf("Tentando escrever em _event_buffer[%d + %d]...\n",
           _next_buffer_idx, EVENT_BUFFER_HEADER_SIZE);
    memset(_event_buffer + _next_buffer_idx + EVENT_BUFFER_HEADER_SIZE, 0, args_size);

    free(_event_buffer);
    return 0;
}
