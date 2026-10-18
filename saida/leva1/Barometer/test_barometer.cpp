/**
 * @file test_barometer.cpp
 * @brief Verificação formal do módulo Barometer do PX4
 * Dissertação Mestrado - Verificação Formal PX4 com ESBMC
 * Módulo: src/lib/sensor_calibration/Barometer.cpp
 *
 * Contexto SAGRES: O barômetro é o sensor primário de altitude do drone.
 * Erros de calibração ou offset inválido causam estimativa incorreta de
 * altitude, crítico para missões autônomas de monitoramento na Amazônia.
 */

#include <assert.h>
#include <math.h>
#include <stdint.h>

extern float nondet_float();
extern int   nondet_int();
extern void  __ESBMC_assume(int condition);

// ======== Constantes reais do Barometer.cpp ========
#define MAX_SENSOR_COUNT         4
#define DEFAULT_PRIORITY        50
#define DEFAULT_EXTERNAL_PRIORITY 75
#define CAL_PRIO_UNINITIALIZED  -1
#define OFFSET_CHANGE_THRESHOLD  0.01f   // fabsf(_offset - offset) > 0.01f

// ======== Estrutura simplificada do Barometer ========
typedef struct {
    float    offset;
    float    thermal_offset;
    int32_t  priority;
    int8_t   calibration_index;
    uint32_t calibration_count;
    int      external;
} Barometer_t;

// ======== Funções extraídas do Barometer.cpp ========

/**
 * Simula set_offset():
 * Aceita apenas se PX4_ISFINITE(offset) && |_offset - offset| > 0.01
 */
int set_offset(Barometer_t *baro, float new_offset)
{
    if (!isnan(new_offset) && !isinf(new_offset) && fabsf(baro->offset - new_offset) > OFFSET_CHANGE_THRESHOLD) {
        if (!isnan(new_offset) && !isinf(new_offset)) {
            baro->offset = new_offset;
            baro->calibration_count++;
            return 1;  // true
        }
    }
    return 0;  // false
}

/**
 * Simula validação de prioridade em ParametersLoad():
 * if (priority < 0 || priority > 100) → reset para default
 */
int32_t validate_priority(int32_t priority, int external)
{
    if (priority < 0 || priority > 100) {
        return external ? DEFAULT_EXTERNAL_PRIORITY : DEFAULT_PRIORITY;
    }
    return priority;
}

/**
 * Simula set_calibration_index():
 * válido apenas se index em [0, MAX_SENSOR_COUNT)
 */
int set_calibration_index(Barometer_t *baro, int index)
{
    if (index >= 0 && index < MAX_SENSOR_COUNT) {
        baro->calibration_index = (int8_t)index;
        return 1;  // true
    }
    return 0;  // false
}

/**
 * Simula Correct() — aplicação do offset e correção térmica:
 * corrected = raw_pressure - offset - thermal_offset
 */
float correct_pressure(float raw_pressure, float offset, float thermal_offset)
{
    return raw_pressure - offset - thermal_offset;
}

/**
 * Simula Reset():
 * offset=0, thermal_offset=0, priority=default, calibration_index=-1
 */
void reset_barometer(Barometer_t *baro)
{
    baro->offset             = 0.0f;
    baro->thermal_offset     = 0.0f;
    baro->priority           = baro->external ? DEFAULT_EXTERNAL_PRIORITY : DEFAULT_PRIORITY;
    baro->calibration_index  = -1;
    baro->calibration_count  = 0;
}

// ======== TESTES ========

/**
 * TESTE 1: set_offset() só aceita valores finitos com mudança significativa
 * Baseado em: if (PX4_ISFINITE(offset) && fabsf(_offset - offset) > 0.01f)
 */
void test_offset_acceptance() {
    Barometer_t baro = {0.0f, 0.0f, DEFAULT_PRIORITY, 0, 0, 0};
    float new_offset = nondet_float();
    float old_offset = nondet_float();

    __ESBMC_assume(!isnan(old_offset) && !isinf(old_offset));
    __ESBMC_assume(old_offset >= -10000.0f && old_offset <= 10000.0f);

    baro.offset = old_offset;

    int result = set_offset(&baro, new_offset);

    // P1: NaN nunca aceito como offset
    if (isnan(new_offset) || isinf(new_offset)) {
        assert(result == 0);
        assert(baro.offset == old_offset);  // offset não mudou
    }

    // P2: mudança menor que threshold nunca aceita
    if (!isnan(new_offset) && !isinf(new_offset) &&
        fabsf(old_offset - new_offset) <= OFFSET_CHANGE_THRESHOLD) {
        assert(result == 0);
    }

    // P3: offset aceito deve ser finito
    if (result == 1) {
        assert(!isnan(baro.offset));
        assert(!isinf(baro.offset));
    }

    // P4: calibration_count incrementa apenas quando aceito
    if (result == 1) {
        assert(baro.calibration_count == 1);
    } else {
        assert(baro.calibration_count == 0);
    }
}

/**
 * TESTE 2: Prioridade inválida deve ser resetada para default
 * Baseado em: if (priority < 0 || priority > 100) → reset
 */
void test_priority_validation() {
    int32_t priority = nondet_int();
    int     external = nondet_int();

    __ESBMC_assume(external == 0 || external == 1);
    __ESBMC_assume(priority >= -200 && priority <= 200);

    int32_t result = validate_priority(priority, external);

    // P1: resultado sempre em [0, 100]
    assert(result >= 0);
    assert(result <= 100);

    // P2: prioridade válida nunca modificada
    if (priority >= 0 && priority <= 100) {
        assert(result == priority);
    }

    // P3: prioridade inválida → default correto
    if (priority < 0 || priority > 100) {
        if (external) {
            assert(result == DEFAULT_EXTERNAL_PRIORITY);
        } else {
            assert(result == DEFAULT_PRIORITY);
        }
    }
}

/**
 * TESTE 3: Índice de calibração válido apenas em [0, MAX_SENSOR_COUNT)
 * Baseado em: if (index >= 0 && index < MAX_SENSOR_COUNT)
 */
void test_calibration_index_bounds() {
    Barometer_t baro = {0.0f, 0.0f, DEFAULT_PRIORITY, -1, 0, 0};
    int index = nondet_int();

    __ESBMC_assume(index >= -5 && index <= MAX_SENSOR_COUNT + 5);

    int result = set_calibration_index(&baro, index);

    // P1: índice fora do range nunca aceito
    if (index < 0 || index >= MAX_SENSOR_COUNT) {
        assert(result == 0);
        assert(baro.calibration_index == -1);  // não mudou
    }

    // P2: índice válido sempre aceito
    if (index >= 0 && index < MAX_SENSOR_COUNT) {
        assert(result == 1);
        assert(baro.calibration_index == (int8_t)index);
    }
}

/**
 * TESTE 4: Pressão corrigida nunca pode ser NaN para entradas válidas
 * Baseado em: corrected = raw - offset - thermal_offset
 */
void test_pressure_correction_valid() {
    float raw_pressure    = nondet_float();
    float offset          = nondet_float();
    float thermal_offset  = nondet_float();

    __ESBMC_assume(!isnan(raw_pressure)   && !isinf(raw_pressure));
    __ESBMC_assume(!isnan(offset)         && !isinf(offset));
    __ESBMC_assume(!isnan(thermal_offset) && !isinf(thermal_offset));

    // Limites físicos razoáveis para pressão atmosférica (Pa)
    __ESBMC_assume(raw_pressure   >= 50000.0f  && raw_pressure   <= 110000.0f);
    __ESBMC_assume(offset         >= -5000.0f  && offset         <= 5000.0f);
    __ESBMC_assume(thermal_offset >= -1000.0f  && thermal_offset <= 1000.0f);

    float corrected = correct_pressure(raw_pressure, offset, thermal_offset);

    // P1: resultado nunca NaN
    assert(!isnan(corrected));

    // P2: resultado nunca infinito
    assert(!isinf(corrected));

    // P3: pressão corrigida dentro de limites físicos razoáveis
    assert(corrected >= 30000.0f && corrected <= 120000.0f);
}

/**
 * TESTE 5: Após Reset(), estado deve ser consistente e seguro
 * Baseado em: Reset() → offset=0, thermal_offset=0, calibration_index=-1
 */
void test_reset_state_consistent() {
    Barometer_t baro;
    baro.offset            = nondet_float();
    baro.thermal_offset    = nondet_float();
    baro.priority          = nondet_int();
    baro.calibration_index = (int8_t)nondet_int();
    baro.calibration_count = (uint32_t)nondet_int();
    baro.external          = nondet_int() % 2;

    reset_barometer(&baro);

    // P1: offset sempre zero após reset
    assert(baro.offset == 0.0f);

    // P2: thermal_offset sempre zero após reset
    assert(baro.thermal_offset == 0.0f);

    // P3: calibration_index sempre -1 após reset
    assert(baro.calibration_index == -1);

    // P4: calibration_count sempre zero após reset
    assert(baro.calibration_count == 0);

    // P5: prioridade válida após reset
    assert(baro.priority >= 0 && baro.priority <= 100);
}

// ======== MAIN ========
int main() {
    int choice = nondet_int();
    __ESBMC_assume(choice >= 0 && choice < 5);

    switch (choice) {
        case 0: test_offset_acceptance();          break;
        case 1: test_priority_validation();        break;
        case 2: test_calibration_index_bounds();   break;
        case 3: test_pressure_correction_valid();  break;
        case 4: test_reset_state_consistent();     break;
    }

    return 0;
}
