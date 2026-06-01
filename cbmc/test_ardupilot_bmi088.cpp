#include <assert.h>
#include <stdint.h>
#include <stddef.h>

uint8_t nondet_uint8_t();

float ardupilot_bmi088_temperature(uint8_t msb, uint8_t lsb) {
    // Codigo REAL extraido de AP_InertialSensor_BMI088.cpp linha 383-385
    uint16_t temp_uint11 = (msb << 3) | (lsb >> 5);
    int16_t temp_int11 = temp_uint11 > 1023 ? temp_uint11 - 2048 : temp_uint11;
    float temp_degc = temp_int11 * 0.125f + 23;
    // SEM validacao de range -- mesmo bug do PX4
    return temp_degc;
}

int main() {
    uint8_t msb = nondet_uint8_t();
    uint8_t lsb = nondet_uint8_t();
    float temp = ardupilot_bmi088_temperature(msb, lsb);
    // PROPRIEDADE: range operacional BMI088 [-40, +85] graus C
    assert(temp >= -40.0f && temp <= 85.0f);
    return 0;
}
