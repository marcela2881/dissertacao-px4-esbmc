#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define LOG_BUFFER_SIZE 127
#define CHUNK_SIZE 50
int main() {
    char log_text[LOG_BUFFER_SIZE];
    char chunk_text[CHUNK_SIZE];
    size_t offset = nondet_size_t();  // simulates strlen(log_text) WITHOUT guaranteed invariant
    // Deliberately omits the explicit bounded null-termination
    // that the real MavlinkStatustextHandler enforces after every write.
    size_t max_to_add = LOG_BUFFER_SIZE - offset - 1;
    if (max_to_add > CHUNK_SIZE) {
        max_to_add = CHUNK_SIZE;
    }
    memcpy(log_text + offset, chunk_text, max_to_add);
    return 0;
}
