#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define BUFFER_SIZE 500
int main() {
    char dump_buffer[BUFFER_SIZE];
    char src[BUFFER_SIZE];
    size_t dump_len = nondet_size_t();
    size_t write_len = nondet_size_t();
    if (write_len > BUFFER_SIZE - dump_len) {
        write_len = BUFFER_SIZE - dump_len;
    }
    memcpy(dump_buffer + dump_len, src, write_len);
    return 0;
}
