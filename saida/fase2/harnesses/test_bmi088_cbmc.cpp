#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <climits>

int nondet_int();
uint8_t nondet_uint8_t();

static const uint16_t FIFO_SIZE = 1024;

static int16_t combine(uint8_t msb, uint8_t lsb) {
    return (int16_t)((msb << 8u) | lsb);
}

float updateTemperature(uint8_t temp_msb, uint8_t temp_lsb) {
    uint16_t Temp_uint11 = (temp_msb * 8) + (temp_lsb / 32);
    int16_t Temp_int11 = (Temp_uint11 > 1023) ? (int16_t)(Temp_uint11 - 2048) : (int16_t)Temp_uint11;
    return (Temp_int11 * 0.125f) + 23.0f;
}

uint16_t fifoReadCount(uint8_t fifo_length_0, uint8_t fifo_length_1) {
    const uint8_t FIFO_LENGTH_1_MASKED = fifo_length_1 & 0x3F;
    return (uint16_t)combine(FIFO_LENGTH_1_MASKED, fifo_length_0);
}

int main() {
    uint8_t temp_msb = nondet_uint8_t();
    uint8_t temp_lsb = nondet_uint8_t();
    float temperature = updateTemperature(temp_msb, temp_lsb);
    assert(temperature >= -40.0f && temperature <= 85.0f);

    uint8_t fifo_len_0 = nondet_uint8_t();
    uint8_t fifo_len_1 = nondet_uint8_t();
    uint16_t count = fifoReadCount(fifo_len_0, fifo_len_1);
    assert(count <= FIFO_SIZE);

    return 0;
}
