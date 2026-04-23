/**
 * @file test_rc_update.cpp
 * @brief Verificação formal do módulo RC Update do PX4
 * Dissertação Mestrado - Verificação Formal PX4 com ESBMC
 * Módulo: src/modules/rc_update/rc_update.cpp
 */

#include <assert.h>
#include <math.h>
#include <stdint.h>

extern float nondet_float();
extern int   nondet_int();
extern void  __ESBMC_assume(int condition);

// ======== Constantes reais do rc_update.cpp ========
#define RC_MAX_CHAN_COUNT   18
#define RC_MIN_CHANNELS      4   // mínimo para sinal válido

// Switch positions (manual_control_switches_s)
#define SWITCH_POS_NONE    0
#define SWITCH_POS_ON      1
#define SWITCH_POS_OFF     2
#define SWITCH_POS_MIDDLE  3

// ======== Funções extraídas do rc_update.cpp ========

/**
 * Simula get_rc_value() com math::constrain
 * return math::constrain(_rc.channels[func], min_value, max_value)
 */
float get_rc_value(float channel_value, float min_value, float max_value)
{
    // math::constrain
    if (channel_value < min_value) return min_value;
    if (channel_value > max_value) return max_value;
    return channel_value;
}

/**
 * Simula interpolação piecewise linear de calibração de canal RC:
 * math::interpolateNXY(value, {min, trim, max}, {-1.f, 0.f, 1.f})
 */
float calibrate_rc_channel(float value, float min, float trim, float max)
{
    if (max <= min) return 0.0f;  // parâmetros inválidos

    float result;

    if (value <= trim) {
        if (trim <= min) return -1.0f;
        result = -1.0f + (value - min) / (trim - min);
    } else {
        if (max <= trim) return 1.0f;
        result = (value - trim) / (max - trim);
    }

    // constrain para [-1, 1]
    if (result < -1.0f) result = -1.0f;
    if (result >  1.0f) result =  1.0f;

    return result;
}

/**
 * Simula getRCSwitchOnOffPosition():
 * float value = 0.5f * channel + 0.5f  (rescale [-1,1] -> [0,1])
 * return (value > threshold) ? ON : OFF
 */
int get_switch_on_off(float channel_value, float threshold)
{
    float value = 0.5f * channel_value + 0.5f;  // rescale [-1,1] -> [0,1]

    if (threshold < 0.0f) {
        value = -value;
    }

    return (value > threshold) ? SWITCH_POS_ON : SWITCH_POS_OFF;
}

/**
 * Simula lógica de detecção de sinal perdido:
 * channel_count < 4 → signal_lost = true
 */
int detect_signal_lost(int channel_count, int rc_lost_flag, int rc_failsafe_flag)
{
    if (rc_lost_flag || rc_failsafe_flag || channel_count < RC_MIN_CHANNELS) {
        return 1;  // signal lost
    }
    return 0;
}

/**
 * Simula correção do trim do throttle:
 * if (trim == min) new_trim = (min + max) / 2
 */
uint16_t correct_throttle_trim(uint16_t min, uint16_t trim, uint16_t max)
{
    if (trim == min && min < max) {
        return (min + max) / 2;
    }
    return trim;
}

// ======== TESTES ========

/**
 * TESTE 1: get_rc_value() deve sempre retornar valor em [-1, 1]
 * Função crítica: controla roll, pitch, yaw, throttle
 */
void test_rc_value_range() {
    float channel = nondet_float();
    __ESBMC_assume(!isnan(channel) && !isinf(channel));
    __ESBMC_assume(channel >= -10.0f && channel <= 10.0f);

    float result = get_rc_value(channel, -1.0f, 1.0f);

    // P1: nunca abaixo de -1
    assert(result >= -1.0f);

    // P2: nunca acima de 1
    assert(result <= 1.0f);

    // P3: resultado não pode ser NaN
    assert(!isnan(result));
}

/**
 * TESTE 2: Calibração de canal RC deve produzir valor em [-1, 1]
 * Baseado em: math::interpolateNXY(value, {min, trim, max}, {-1, 0, 1})
 */
void test_channel_calibration_range() {
    float value = nondet_float();
    float min   = nondet_float();
    float trim  = nondet_float();
    float max   = nondet_float();

    __ESBMC_assume(!isnan(value) && !isinf(value));
    __ESBMC_assume(min  >= 800.0f  && min  <= 1200.0f);
    __ESBMC_assume(trim >= 1400.0f && trim <= 1600.0f);
    __ESBMC_assume(max  >= 1800.0f && max  <= 2200.0f);
    __ESBMC_assume(min < trim && trim < max);
    __ESBMC_assume(value >= min && value <= max);

    float result = calibrate_rc_channel(value, min, trim, max);

    // P1: resultado em [-1, 1]
    assert(result >= -1.0f);
    assert(result <= 1.0f);

    // P2: resultado não pode ser NaN
    assert(!isnan(result));

    // P3: valor no trim deve resultar em ~0
    float at_trim = calibrate_rc_channel(trim, min, trim, max);
    assert(at_trim >= -0.01f && at_trim <= 0.01f);
}

/**
 * TESTE 3: Rescale do switch [-1,1] -> [0,1] nunca pode sair do range
 * Baseado em: float value = 0.5f * channel + 0.5f
 */
void test_switch_rescale_range() {
    float channel   = nondet_float();
    float threshold = nondet_float();

    __ESBMC_assume(!isnan(channel) && !isinf(channel));
    __ESBMC_assume(channel   >= -1.0f && channel   <= 1.0f);
    __ESBMC_assume(threshold >= 0.0f  && threshold <= 1.0f);

    float rescaled = 0.5f * channel + 0.5f;

    // P1: rescale nunca abaixo de 0
    assert(rescaled >= 0.0f);

    // P2: rescale nunca acima de 1
    assert(rescaled <= 1.0f);

    // P3: resultado do switch deve ser ON ou OFF (nunca NONE para canal mapeado)
    int sw = get_switch_on_off(channel, threshold);
    assert(sw == SWITCH_POS_ON || sw == SWITCH_POS_OFF);
}

/**
 * TESTE 4: Menos de 4 canais deve SEMPRE indicar sinal perdido
 * Baseado em: if (channel_count < 4) signal_lost = true
 */
void test_signal_lost_min_channels() {
    int channel_count = nondet_int();
    int rc_lost       = nondet_int();
    int rc_failsafe   = nondet_int();

    __ESBMC_assume(channel_count >= 0 && channel_count <= RC_MAX_CHAN_COUNT);
    __ESBMC_assume(rc_lost == 0 || rc_lost == 1);
    __ESBMC_assume(rc_failsafe == 0 || rc_failsafe == 1);

    int lost = detect_signal_lost(channel_count, rc_lost, rc_failsafe);

    // P1: menos de 4 canais → sempre sinal perdido
    if (channel_count < RC_MIN_CHANNELS) {
        assert(lost == 1);
    }

    // P2: rc_lost flag → sempre sinal perdido
    if (rc_lost) {
        assert(lost == 1);
    }

    // P3: rc_failsafe flag → sempre sinal perdido
    if (rc_failsafe) {
        assert(lost == 1);
    }
}

/**
 * TESTE 5: Trim corrigido do throttle deve estar entre min e max
 * Baseado em: new_throttle_trim = (min + max) / 2
 */
void test_throttle_trim_correction() {
    uint16_t min  = (uint16_t)nondet_int();
    uint16_t max  = (uint16_t)nondet_int();
    uint16_t trim = (uint16_t)nondet_int();

    __ESBMC_assume(min  >= 900  && min  <= 1200);
    __ESBMC_assume(max  >= 1800 && max  <= 2100);
    __ESBMC_assume(min < max);
    __ESBMC_assume(trim == min);  // caso que dispara a correção

    uint16_t new_trim = correct_throttle_trim(min, trim, max);

    // P1: trim corrigido deve estar entre min e max
    assert(new_trim >= min);
    assert(new_trim <= max);

    // P2: deve ser exatamente o ponto médio
    assert(new_trim == (min + max) / 2);
}

// ======== MAIN ========
int main() {
    int choice = nondet_int();
    __ESBMC_assume(choice >= 0 && choice < 5);

    switch (choice) {
        case 0: test_rc_value_range();            break;
        case 1: test_channel_calibration_range(); break;
        case 2: test_switch_rescale_range();      break;
        case 3: test_signal_lost_min_channels();  break;
        case 4: test_throttle_trim_correction();  break;
    }

    return 0;
}
