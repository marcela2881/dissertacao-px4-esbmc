#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define LINEBUF_SIZE 20
#define READBUF_SIZE 30

static char _readbuf[READBUF_SIZE];
static char _raw_speed[5];
static char _raw_angle[5];
static char _raw_status[2];
static char _raw_checksum[3];
static int _byte_counter = 0;
static int _msg_byte_counter = 0;
static int _msg_part_counter = 0;

void collect_call(const char *readbuf, int ret) {
    for (int i = 0; i < ret; i++) {
        // linha real ft7_technologies.cpp — sem checagem de limite
        _readbuf[_byte_counter] = readbuf[i];
        _byte_counter += 1;
        if (readbuf[i] == '\n') {
            _byte_counter = 0;
            _msg_byte_counter = 0;
            _msg_part_counter = 0;
        } else if (readbuf[i] == ',' || readbuf[i] == '$') {
            _msg_part_counter++;
            _msg_byte_counter = 0;
        } else {
            if (_msg_part_counter == 3) _raw_speed[_msg_byte_counter] = readbuf[i];
            else if (_msg_part_counter == 4) _raw_angle[_msg_byte_counter] = readbuf[i];
            else if (_msg_part_counter == 5) _raw_status[_msg_byte_counter] = readbuf[i];
            else if (_msg_part_counter == 6) _raw_checksum[_msg_byte_counter] = readbuf[i];
            _msg_byte_counter++;
        }
    }
}

int main() {
    // Aloca _readbuf no heap para detecção precisa pelo ASan
    char *readbuf_heap = (char*)malloc(READBUF_SIZE);
    memset(readbuf_heap, 0, READBUF_SIZE);
    // Substitui o global por heap-allocated
    char *readbuf_dyn = (char*)malloc(READBUF_SIZE);
    memset(readbuf_dyn, 'X', READBUF_SIZE); // sem '\n' — simula linha mal formada

    // Duas chamadas de collect() sem '\n' = 19+19=38 bytes > READBUF_SIZE=30
    char linebuf[LINEBUF_SIZE - 1];
    memset(linebuf, 'X', sizeof(linebuf)); // sem '\n'

    printf("READBUF_SIZE=%d, duas chamadas de %d bytes cada\n",
           READBUF_SIZE, (int)sizeof(linebuf));
    printf("Total esperado: %d bytes > %d = overflow\n",
           2*(int)sizeof(linebuf), READBUF_SIZE);

    // Usamos malloc para _readbuf para ASan detectar o overflow
    char *_readbuf_heap = (char*)malloc(READBUF_SIZE);
    int counter = 0;

    // Primeira chamada
    for (int i = 0; i < (int)sizeof(linebuf); i++) {
        _readbuf_heap[counter] = linebuf[i]; // sem checagem
        counter++;
    }
    printf("Apos 1a chamada: counter=%d\n", counter);

    // Segunda chamada
    for (int i = 0; i < (int)sizeof(linebuf); i++) {
        _readbuf_heap[counter] = linebuf[i]; // overflow aqui
        counter++;
    }
    printf("Apos 2a chamada: counter=%d\n", counter);

    free(_readbuf_heap);
    free(readbuf_heap);
    free(readbuf_dyn);
    return 0;
}
