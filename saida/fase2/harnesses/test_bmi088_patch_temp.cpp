#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <climits>

uint8_t nondet_uint8_t();

float updateTemperature_PATCHED(uint8_t temp_msb, uint8_t temp_lsb) {
    uint16_t Temp_uint11 = (temp_msb * 8) + (temp_lsb / 32);
    int16_t Temp_int11 = (Temp_uint11 > 1023) ? (int16_t)(Temp_uint11 - 2048) : (int16_t)Temp_uint11;
    float temperature = (Temp_int11 * 0.125f) + 23.0f;
    // PATCH LC #3: guard de range operacional
    if (temperature < -40.0f || temperature > 85.0f) {
        return -999.0f; // valor sentinela: fora de range, rejeitar
    }
    return temperature;
}

int main() {
    uint8_t temp_msb = nondet_uint8_t();
    uint8_t temp_lsb = nondet_uint8_t();
    float temperature = updateTemperature_PATCHED(temp_msb, temp_lsb);
    // PROPRIEDADE: se retornou valor valido, deve estar no range
    if (temperature != -999.0f) {
        assert(temperature >= -40.0f && temperature <= 85.0f);
    }
    return 0;
}
