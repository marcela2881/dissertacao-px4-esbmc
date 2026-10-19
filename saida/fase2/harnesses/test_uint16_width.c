#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define BUFFER_SIZE 200
int main() {
    char dump_buffer[BUFFER_SIZE];
    char src[BUFFER_SIZE];
    uint16_t dump_len = nondet_uint16();
    uint16_t write_len = nondet_uint16();
    if (write_len > (uint16_t)(BUFFER_SIZE - dump_len)) {
        write_len = (uint16_t)(BUFFER_SIZE - dump_len);
    }
    memcpy(dump_buffer + dump_len, src, write_len);
    return 0;
}
