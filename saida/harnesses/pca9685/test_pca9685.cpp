/*
 * Harness de verificação — driver PCA9685 do PX4 (src/drivers/pca9685_pwm_out)
 *
 * O código do driver NÃO é copiado: este arquivo é verificado junto com o
 * PCA9685.cpp ORIGINAL do PX4 (commit fixado), e só as dependências externas
 * (I2C, log, configuração de placa) são substituídas pelos stubs em saida/stubs/.
 *
 * Cada teste é escolhido de forma não determinística em main(), como nos
 * harnesses do artigo anterior.
 *
 * PRÉ-CONDIÇÕES: vêm do próprio PX4 (module.yaml do driver e do wrapper main.cpp),
 * e não de escolha arbitrária:
 *   - PCA9685_PWM_FREQ: [23.8, 1525.87] Hz            (module.yaml)
 *   - pulso padrão (min/max/disarmed/failsafe): [800, 2200] us (module.yaml)
 *   - num_outputs <= 16: o wrapper satura antes de chamar updateRAW (main.cpp)
 *
 * PROPRIEDADES DE DATASHEET (NXP PCA9685, a conferir e citar com página):
 *   - DS1: contador PWM de 12 bits -> valor bruto em [0, 4095]; 4096 é tratado
 *          pelo driver como "full ON".
 *   - DS2: LEDn_OFF_H: bits [3:0] são o nibble alto da contagem e o bit 4 é
 *          "full OFF"; num valor normal, o byte deve ser <= 0x0F.
 *   - DS3: PRE_SCALE tem valor mínimo 3 (0x03) e máximo 255 (0xFF).
 *
 * >>> As propriedades DS1–DS3 são uma PROPOSTA inicial. Confirme cada uma no
 * >>> datasheet (seção e página) antes de usar os resultados.
 */

#include <cstdint>
#include <cmath>
#include <cassert>
#include "PCA9685.h"

uint8_t  g_last_send[STUB_I2C_MAX_SEND];
unsigned g_last_send_len = 0;

extern "C" {
int      nondet_int();
uint8_t  nondet_uint8();
uint16_t nondet_uint16();
unsigned nondet_uint();
float    nondet_float();
void     __VERIFIER_assume(int);   /* aceito pelo ESBMC e pelo CBMC: mesma pré-condição nas duas ferramentas */
}

using namespace drv_pca9685_pwm;

/* Expõe métodos protegidos só para o harness, sem alterar o driver. */
class PCA9685Test : public PCA9685
{
public:
	PCA9685Test() : PCA9685(1, 0x40) {}
	int callWritePWM(uint8_t idx, const uint16_t *v, uint8_t n) { return writePWM(idx, v, n); }
};

/* ---------------------------------------------------------------------------
 * T0 — Segurança de memória em updateRAW/writePWM
 * Pré-condição do chamador (wrapper): num_outputs <= 16.
 * Propriedades: automáticas do ESBMC (limites de vetor, ponteiros).
 * ------------------------------------------------------------------------- */
void test_updateRAW_memory()
{
	PCA9685Test dev;
	uint16_t outputs[PCA9685_PWM_CHANNEL_COUNT];
	for (int i = 0; i < PCA9685_PWM_CHANNEL_COUNT; i++) { outputs[i] = nondet_uint16(); }

	unsigned num = nondet_uint();
	__VERIFIER_assume(num <= PCA9685_PWM_CHANNEL_COUNT);

	dev.updateRAW(outputs, num);
}

/* ---------------------------------------------------------------------------
 * T1 — DS2: byte LEDn_OFF_H escrito no chip, para valores normais (1..4095)
 * ------------------------------------------------------------------------- */
void test_off_high_byte()
{
	PCA9685Test dev;
	uint16_t v = nondet_uint16();
	__VERIFIER_assume(v >= 1 && v <= 4095);

	uint16_t outputs[1] = {v};
	dev.updateRAW(outputs, 1);

	/* buffer: [endereço, ON_L, ON_H, OFF_L, OFF_H] */
	assert(g_last_send_len == 5);
	assert(g_last_send[4] <= 0x0F);          /* DS2 */
}

/* ---------------------------------------------------------------------------
 * T2 — DS3: PRE_SCALE escrito em updateFreq, para a faixa do parâmetro
 * ------------------------------------------------------------------------- */
void test_prescale_range()
{
	PCA9685Test dev;
	float freq = nondet_float();
	__VERIFIER_assume(freq >= 23.8f && freq <= 1525.87f);   /* module.yaml */

	int ret = dev.updateFreq(freq);

	(void)ret;
	if (g_last_send_len == 2) {               /* setDivider foi chamado */
		assert(g_last_send[0] == PCA9685_REG_PRE_SCALE);
		assert(g_last_send[1] >= 0x03);       /* DS3: mínimo */
		/* máximo 0xFF é garantido pelo tipo uint8_t */
	}
}

/* ---------------------------------------------------------------------------
 * T3 — DS1: pulso padrão -> valor bruto de 12 bits
 * Caminho real: updateFreq(freq) define currentFreq; calcRawFromPulse(pulso)
 * converte microssegundos em contagem do chip.
 * ------------------------------------------------------------------------- */
void test_pulse_to_raw()
{
	PCA9685Test dev;
	float freq = nondet_float();
	__VERIFIER_assume(freq >= 23.8f && freq <= 1525.87f);   /* module.yaml */
	dev.updateFreq(freq);

	uint16_t pulse = nondet_uint16();
	__VERIFIER_assume(pulse >= 800 && pulse <= 2200);       /* module.yaml */

	uint16_t raw = dev.calcRawFromPulse(pulse);
	assert(raw <= 4096);                     /* DS1 (4096 = full ON) */
}

/* ---------------------------------------------------------------------------
 * T4 — Robustez de updateFreq fora da faixa do parâmetro (sem pré-condição)
 * Objetivo: ver se conversões float -> inteiro podem ter comportamento indefinido.
 * Rodar com --float-overflow-check / checagens de conversão do ESBMC.
 * ------------------------------------------------------------------------- */
void test_updateFreq_any()
{
	PCA9685Test dev;
	float freq = nondet_float();
	dev.updateFreq(freq);
}

int main()
{
#ifdef TEST_CHOICE
	int choice = TEST_CHOICE;                 /* rodar um teste por vez: -DTEST_CHOICE=n */
#else
	int choice = nondet_int();
	__VERIFIER_assume(choice >= 0 && choice <= 4);
#endif

	switch (choice) {
	case 0: test_updateRAW_memory(); break;
	case 1: test_off_high_byte();    break;
	case 2: test_prescale_range();   break;
	case 3: test_pulse_to_raw();     break;
	case 4: test_updateFreq_any();   break;
	}

	return 0;
}
