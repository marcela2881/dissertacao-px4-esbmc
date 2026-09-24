#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define BUFFER_SIZE 200
int main() {
    char dump_buffer[BUFFER_SIZE];
    char src[BUFFER_SIZE];
    size_t dump_len = nondet_size_t();  // starts nondet, NOT zero (already-corrupted state)
    size_t total_len = nondet_size_t();
    while (total_len > 0) {
        size_t write_len = total_len;
        if (write_len > BUFFER_SIZE - dump_len) {
            write_len = BUFFER_SIZE - dump_len;
        }
        memcpy(dump_buffer + dump_len, src, write_len);
        dump_len += write_len;
        total_len -= write_len;
    }
    return 0;
}
