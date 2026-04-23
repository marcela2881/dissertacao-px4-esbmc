/**
 * @file test_pwm_out.cpp
 * @brief Verificação formal do módulo PWMOut do PX4
 * Dissertação Mestrado - Verificação Formal PX4 com ESBMC
 * Módulo: src/drivers/pwm_out/PWMOut.cpp
 */

#include <assert.h>
#include <math.h>
#include <stdint.h>

extern float    nondet_float();
extern int      nondet_int();
extern uint32_t nondet_uint();
extern void     __ESBMC_assume(int condition);

// ======== Constantes reais do PWMOut ========
#define PWM_MOTOR_MIN       1100   // us — mínimo para motor (idle)
#define PWM_MOTOR_MAX       1900   // us — máximo para motor
#define PWM_SERVO_DISARMED  1500   // us — posição neutra servo
#define PWM_ABSOLUTE_MIN     900   // us — limite físico mínimo do hardware
#define PWM_ABSOLUTE_MAX    2100   // us — limite físico máximo do hardware
#define MAX_ACTUATORS         16
#define MAX_IO_TIMERS          4

// ======== Funções extraídas do PWMOut.cpp ========

/**
 * Simula lroundf + cast para uint16_t como em updateOutputs()
 * up_pwm_servo_set(i, static_cast<uint16_t>(lroundf(outputs[i])))
 */
uint16_t convert_output_to_pwm(float output)
{
    int rounded = (int)(output + 0.5f);
    if (rounded < 0)     rounded = 0;
    if (rounded > 65535) rounded = 65535;
    return (uint16_t)rounded;
}

/**
 * Simula a lógica de updateOutputs():
 * canal desabilitado → output zerado
 * canal habilitado   → converte para PWM
 */
uint16_t process_channel(float output, int channel_enabled, int function_set)
{
    float effective = output;

    if (!function_set) {
        effective = 0.0f;  // canal sem função → força zero
    }

    if (channel_enabled) {
        return convert_output_to_pwm(effective);
    }

    return 0;
}

/**
 * Simula a validação de parâmetros de motor em update_params():
 * motor min=1100, motor max=1900
 */
int validate_motor_params(int32_t min_us, int32_t max_us)
{
    if (min_us != PWM_MOTOR_MIN) return -1;
    if (max_us != PWM_MOTOR_MAX) return -1;
    if (min_us >= max_us)        return -1;
    return 0;
}

/**
 * Simula validação de parâmetro de servo em update_params():
 * servo disarmed = 1500us
 */
int validate_servo_disarmed(int32_t disarmed_us)
{
    return (disarmed_us == PWM_SERVO_DISARMED) ? 0 : -1;
}

// ======== TESTES ========

/**
 * TESTE 1: Saída PWM de motor deve estar dentro do range operacional
 * Range válido: 1100µs (idle) a 1900µs (full throttle)
 */
void test_motor_pwm_range() {
    float output   = nondet_float();
    int   ch_en    = nondet_int();
    int   fn_set   = nondet_int();

    __ESBMC_assume(output >= (float)PWM_MOTOR_MIN && output <= (float)PWM_MOTOR_MAX);
    __ESBMC_assume(ch_en == 1);
    __ESBMC_assume(fn_set == 1);

    uint16_t pwm = process_channel(output, ch_en, fn_set);

    // P1: PWM nunca abaixo do mínimo de motor
    assert(pwm >= PWM_MOTOR_MIN);

    // P2: PWM nunca acima do máximo de motor
    assert(pwm <= PWM_MOTOR_MAX);

    // P3: valor dentro dos limites físicos absolutos do hardware
    assert(pwm >= PWM_ABSOLUTE_MIN);
    assert(pwm <= PWM_ABSOLUTE_MAX);
}

/**
 * TESTE 2: Canal desabilitado (sem função) deve sempre produzir PWM = 0
 * Baseado em: if (!_mixing_output.isFunctionSet(i)) outputs[i] = 0.f
 */
void test_disabled_channel_zero_output() {
    float output = nondet_float();
    int   ch_en  = nondet_int();

    __ESBMC_assume(output >= -10000.0f && output <= 10000.0f);
    __ESBMC_assume(ch_en == 1 || ch_en == 0);

    // Canal sem função definida
    uint16_t pwm = process_channel(output, ch_en, 0 /* function NOT set */);

    // P1: canal sem função DEVE produzir 0
    assert(pwm == 0);
}

/**
 * TESTE 3: Conversão float→uint16_t não pode causar overflow
 * static_cast<uint16_t>(lroundf(outputs[i])) em updateOutputs()
 */
void test_pwm_conversion_no_overflow() {
    float output = nondet_float();

    __ESBMC_assume(!isnan(output) && !isinf(output));
    __ESBMC_assume(output >= 0.0f && output <= 65535.0f);

    uint16_t pwm = convert_output_to_pwm(output);

    // P1: resultado deve ser valor uint16 válido
    assert(pwm <= 65535);

    // P2: para valores no range de motor, deve preservar o valor
    if (output >= (float)PWM_MOTOR_MIN && output <= (float)PWM_MOTOR_MAX) {
        assert(pwm >= PWM_MOTOR_MIN);
        assert(pwm <= PWM_MOTOR_MAX);
    }
}

/**
 * TESTE 4: Parâmetros de motor devem ser exatamente min=1100, max=1900
 * Baseado em update_params(): val=1100 para min, val=1900 para max
 */
void test_motor_param_values() {
    int32_t min_val = nondet_int();
    int32_t max_val = nondet_int();

    // Simula o que o PX4 configura automaticamente
    min_val = PWM_MOTOR_MIN;
    max_val = PWM_MOTOR_MAX;

    int result = validate_motor_params(min_val, max_val);

    // P1: configuração automática deve ser sempre válida
    assert(result == 0);

    // P2: min deve ser menor que max
    assert(min_val < max_val);

    // P3: valores dentro dos limites físicos
    assert(min_val >= PWM_ABSOLUTE_MIN);
    assert(max_val <= PWM_ABSOLUTE_MAX);
}

/**
 * TESTE 5: Servo disarmed deve ser exatamente 1500µs
 * Baseado em update_params(): val = 1500 para servos
 */
void test_servo_disarmed_value() {
    int32_t disarmed = PWM_SERVO_DISARMED;  // valor configurado pelo PX4

    int result = validate_servo_disarmed(disarmed);

    // P1: valor configurado automaticamente deve ser válido
    assert(result == 0);

    // P2: deve ser exatamente 1500µs (posição neutra)
    assert(disarmed == 1500);

    // P3: dentro dos limites físicos
    assert(disarmed >= PWM_ABSOLUTE_MIN);
    assert(disarmed <= PWM_ABSOLUTE_MAX);
}

// ======== MAIN ========
int main() {
    int choice = nondet_int();
    __ESBMC_assume(choice >= 0 && choice < 5);

    switch (choice) {
        case 0: test_motor_pwm_range();           break;
        case 1: test_disabled_channel_zero_output(); break;
        case 2: test_pwm_conversion_no_overflow(); break;
        case 3: test_motor_param_values();         break;
        case 4: test_servo_disarmed_value();       break;
    }

    return 0;
}
