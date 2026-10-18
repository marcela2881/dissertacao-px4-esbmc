/**
 * @file test_battery.cpp
 * @brief Verificação formal do módulo Battery do PX4
 * Dissertação Mestrado - Verificação Formal PX4 com ESBMC
 * Módulo: src/lib/battery/battery.cpp
 */

#include <assert.h>
#include <math.h>
#include <stdint.h>

extern float nondet_float();
extern int   nondet_int();
extern void  __ESBMC_assume(int condition);

// ======== Constantes reais do battery.cpp ========
#define BAT_TEMP_MAX       75.0f
#define LITHIUM_BATTERY_RECOGNITION_VOLTAGE 3.0f

// Níveis de warning (battery_status_s)
#define WARNING_NONE       0
#define WARNING_LOW        1
#define WARNING_CRITICAL   2
#define WARNING_EMERGENCY  3

// Faults
#define FAULT_SPIKES           0
#define FAULT_OVER_TEMPERATURE 5

// ======== Funções extraídas do battery.cpp ========

uint8_t determineWarning(float state_of_charge,
                         float low_thr,
                         float crit_thr,
                         float emergen_thr)
{
    if (state_of_charge < emergen_thr) {
        return WARNING_EMERGENCY;
    } else if (state_of_charge < crit_thr) {
        return WARNING_CRITICAL;
    } else if (state_of_charge < low_thr) {
        return WARNING_LOW;
    } else {
        return WARNING_NONE;
    }
}

uint16_t determineFaults(float temperature_c,
                         float voltage_v,
                         int   n_cells,
                         float v_charged)
{
    uint16_t faults = 0;

    if ((n_cells > 0) && (voltage_v > (n_cells * v_charged * 1.05f))) {
        faults |= (1 << FAULT_SPIKES);
    }

    if (!isnan(temperature_c) && temperature_c > BAT_TEMP_MAX) {
        faults |= (1 << FAULT_OVER_TEMPERATURE);
    }

    return faults;
}

float computeScale(float v_charged, float cell_voltage_filtered)
{
    if (cell_voltage_filtered <= 0.0f) return 1.0f;

    float scale = v_charged / cell_voltage_filtered;

    if (!isnan(scale) && !isinf(scale)) {
        if (scale < 1.0f) scale = 1.0f;
        if (scale > 1.3f) scale = 1.3f;
    } else {
        scale = 1.0f;
    }

    return scale;
}

float sumDischarged(float current_a, float dt, float discharged_mah)
{
    if (dt > 1e-6f && fabsf(current_a + 1.0f) > 1e-6f) {
        float loop = (current_a * 1e3f) * (dt / 3600.0f);
        discharged_mah += loop;
    }
    return discharged_mah;
}

// ======== TESTES ========

/**
 * TESTE 1: Warning levels devem respeitar a ordem dos thresholds
 * Se emergen < crit < low, os warnings devem ser consistentes
 */
void test_warning_threshold_order() {
    float soc      = nondet_float();
    float low_thr  = nondet_float();
    float crit_thr = nondet_float();
    float emrg_thr = nondet_float();

    __ESBMC_assume(soc      >= 0.0f && soc      <= 1.0f);
    __ESBMC_assume(emrg_thr >= 0.0f && emrg_thr <  0.1f);
    __ESBMC_assume(crit_thr >  emrg_thr && crit_thr < 0.2f);
    __ESBMC_assume(low_thr  >  crit_thr && low_thr  < 0.4f);

    uint8_t w = determineWarning(soc, low_thr, crit_thr, emrg_thr);

    // P1: resultado deve ser um dos 4 valores válidos
    assert(w == WARNING_NONE     ||
           w == WARNING_LOW      ||
           w == WARNING_CRITICAL ||
           w == WARNING_EMERGENCY);

    // P2: SoC zerado deve sempre gerar emergência
    if (soc < emrg_thr) {
        assert(w == WARNING_EMERGENCY);
    }

    // P3: SoC pleno nunca gera warning
    if (soc >= low_thr) {
        assert(w == WARNING_NONE);
    }
}

/**
 * TESTE 2: Detecção de over-temperature
 * Temperatura acima de BAT_TEMP_MAX (75°C) deve gerar fault
 */
void test_over_temperature_fault() {
    float temp    = nondet_float();
    float voltage = nondet_float();
    float v_chg   = nondet_float();
    int   n_cells = nondet_int();

    __ESBMC_assume(!isnan(temp) && !isinf(temp));
    __ESBMC_assume(temp    >= -40.0f && temp    <= 120.0f);
    __ESBMC_assume(voltage >= 0.0f   && voltage <= 100.0f);
    __ESBMC_assume(v_chg   >= 3.0f   && v_chg   <= 4.5f);
    __ESBMC_assume(n_cells >= 1      && n_cells <= 12);

    uint16_t faults = determineFaults(temp, voltage, n_cells, v_chg);

    // P1: temperatura acima do limite DEVE gerar fault
    if (temp > BAT_TEMP_MAX) {
        assert(faults & (1 << FAULT_OVER_TEMPERATURE));
    }

    // P2: temperatura normal NÃO deve gerar fault térmico
    if (temp <= BAT_TEMP_MAX) {
        assert(!(faults & (1 << FAULT_OVER_TEMPERATURE)));
    }
}

/**
 * TESTE 3: Scale deve estar sempre entre 1.0 e 1.3
 * Compensação máxima permitida é 30%
 */
void test_scale_bounds() {
    float v_charged          = nondet_float();
    float cell_volt_filtered = nondet_float();

    __ESBMC_assume(v_charged          >= 3.0f && v_charged          <= 4.5f);
    __ESBMC_assume(cell_volt_filtered >= 0.1f && cell_volt_filtered <= 5.0f);

    float scale = computeScale(v_charged, cell_volt_filtered);

    // P1: scale nunca abaixo de 1.0
    assert(scale >= 1.0f);

    // P2: scale nunca acima de 1.3
    assert(scale <= 1.3f);

    // P3: scale nunca NaN
    assert(!isnan(scale));
}

/**
 * TESTE 4: mAh descarregado nunca deve ser negativo
 * para corrente positiva (descarga normal)
 */
void test_discharged_mah_non_negative() {
    float current_a      = nondet_float();
    float dt             = nondet_float();
    float discharged_mah = nondet_float();

    __ESBMC_assume(current_a      >= 0.0f  && current_a      <= 100.0f);
    __ESBMC_assume(dt             >= 1e-5f && dt             <= 2.0f);
    __ESBMC_assume(discharged_mah >= 0.0f  && discharged_mah <= 100000.0f);

    float result = sumDischarged(current_a, dt, discharged_mah);

    // P1: descarga acumulada nunca regride com corrente positiva
    assert(result >= discharged_mah);

    // P2: resultado nunca negativo se início não negativo
    assert(result >= 0.0f);
}

/**
 * TESTE 5: Overvoltage fault
 * Tensão acima de 105% da tensão nominal deve gerar fault de spike
 */
void test_overvoltage_fault() {
    float voltage = nondet_float();
    float v_chg   = nondet_float();
    int   n_cells = nondet_int();

    __ESBMC_assume(voltage  >= 0.0f && voltage <= 100.0f);
    __ESBMC_assume(v_chg    >= 3.0f && v_chg   <= 4.5f);
    __ESBMC_assume(n_cells  >= 1    && n_cells  <= 12);

    float limit = n_cells * v_chg * 1.05f;
    uint16_t faults = determineFaults(20.0f, voltage, n_cells, v_chg);

    // P1: tensão acima do limite DEVE gerar fault de spike
    if (voltage > limit) {
        assert(faults & (1 << FAULT_SPIKES));
    }

    // P2: tensão normal NÃO deve gerar spike
    if (voltage <= limit) {
        assert(!(faults & (1 << FAULT_SPIKES)));
    }
}

// ======== MAIN ========
int main() {
    int choice = nondet_int();
    __ESBMC_assume(choice >= 0 && choice < 5);

    switch (choice) {
        case 0: test_warning_threshold_order();     break;
        case 1: test_over_temperature_fault();      break;
        case 2: test_scale_bounds();                break;
        case 3: test_discharged_mah_non_negative(); break;
        case 4: test_overvoltage_fault();           break;
    }

    return 0;
}
