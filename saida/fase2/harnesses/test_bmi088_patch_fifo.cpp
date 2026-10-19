#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <climits>

uint8_t nondet_uint8_t();

static const uint16_t FIFO_SIZE = 1024;

static int16_t combine(uint8_t msb, uint8_t lsb) {
    return (int16_t)((msb << 8u) | lsb);
}

uint16_t fifoReadCount_PATCHED(uint8_t fifo_length_0, uint8_t fifo_length_1) {
    const uint8_t FIFO_LENGTH_1_MASKED = fifo_length_1 & 0x3F;
    uint16_t count = (uint16_t)combine(FIFO_LENGTH_1_MASKED, fifo_length_0);
    // PATCH LC #3: rejeitar count invalido
    if (count > FIFO_SIZE) {
        return 0; // valor sentinela: FIFO invalido, rejeitar
    }
    return count;
}

int main() {
    uint8_t fifo_len_0 = nondet_uint8_t();
    uint8_t fifo_len_1 = nondet_uint8_t();
    uint16_t count = fifoReadCount_PATCHED(fifo_len_0, fifo_len_1);
    // PROPRIEDADE: count nunca deve exceder FIFO_SIZE
    assert(count <= FIFO_SIZE);
    return 0;
}
