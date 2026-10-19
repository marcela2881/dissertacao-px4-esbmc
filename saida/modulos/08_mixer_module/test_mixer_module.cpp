/*
 * Harness de verificação — mixer de saída do PX4 (src/lib/mixer_module, classe MixingOutput)
 *
 * O código NÃO é copiado: este arquivo inclui o mixer_module.cpp ORIGINAL do PX4
 * (commit fixado). Só o ambiente é substituído pelos stubs em saida/stubs/
 * (uORB, parâmetros, fila de trabalho, contadores perf). O output_functions.hpp é
 * o gerado pelo próprio script do PX4 a partir do output_functions.yaml original
 * (stubs/mixer_module/). math::interpolate e interpolateNXY vêm do
 * src/lib/mathlib ORIGINAL.
 *
 * Recorte: o caminho que entrega os valores ao driver de saída,
 *   limitAndUpdateOutputs -> output_limit_calc -> output_limit_calc_single
 *   e, em terminação, actualFailsafeValue.
 * O cálculo dos valores pelas funções (motores, servos, RC...) e as assinaturas
 * uORB ficam fora: os valores de entrada (outputs[]) são livres (qualquer float,
 * inclusive NaN e infinito). É por esse caminho que o failsafe da RoboClaw chega
 * ao driver sem limite (caso R1).
 *
 * PRÉ-CONDIÇÕES (origem anotada):
 *   - parâmetros por canal, faixas do src/drivers/pwm_out/module.yaml
 *     (standard_params): disarmed [800,2200], min [800,1400], center [800,2200],
 *     max [1600,2200], failsafe [800,2200] ou "não definido" (UINT16_MAX, valor
 *     inicial do construtor: usa o failsafe padrão da função).
 *   - 2 canais (max_num_outputs = 2): o código trata cada canal da mesma forma
 *     (laço sobre i); o índice só entra na máscara de inversão (bit i).
 *   - PWM_MAIN_REV (máscara de inversão): qualquer valor.
 *   - função do canal (PWM_MAIN_FUNCn): qualquer valor; o centro só é usado para
 *     servos, trem de pouso com roda e gimbal (mixer_module.cpp, linhas 551-556).
 *
 * PROPRIEDADES (especificação: documentação dos parâmetros PWM_MAIN_MINn/MAXn/
 * DISn/FAILn gerada do module.yaml, e comentários do próprio mixer_module.cpp):
 *   MX1 (T0, T1): valor finito -> saída em [min, max] do canal; valor inválido
 *       (NaN/infinito) -> valor de desarmado ("check for invalid / disabled
 *       channels"). T0 sem hipótese sobre o centro; T1 com centro em [min, max].
 *   MX2 (T2): em terminação, a saída é o failsafe configurado; sem failsafe
 *       configurado, o failsafe padrão da função limitado (sem função: desarmado).
 *   MX3 (T3): em terminação, a saída fica em [min, max]? (pergunta, não
 *       propriedade documentada: o código repassa o failsafe sem limitar; é a
 *       origem do caso RoboClaw R1).
 *   MX4 (T4): em lockdown ou kill, a saída é o valor de desarmado.
 *   MX5 (T5): máquina de estados de armar (OFF/RAMP/ON), qualquer estado, tempo
 *       e pré-arme: a saída fica entre o desarmado e [min, max] (rampa) e é finita.
 */

#define STUB_MATHLIB_ORIGINAL 1          /* math:: vem do src/lib/mathlib original */
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cfloat>
#include <climits>
#include <cassert>
#include <new>

extern "C" {
int      nondet_int();
unsigned nondet_uint();
uint16_t nondet_uint16();
uint64_t nondet_uint64();
float    nondet_float();
bool     nondet_bool();
void     __VERIFIER_assume(int);   /* aceito pelo ESBMC e pelo CBMC */
}

#include <drivers/drv_hrt.h>
#include <mathlib/math/Functions.hpp>   /* ORIGINAL: math::interpolate, interpolateNXY */

/* acesso aos membros privados do MixingOutput, só para montar o estado e checar */
#define private public
#define protected public
#include "mixer_module.cpp"             /* ORIGINAL do PX4 (unidade de tradução única) */
#undef private
#undef protected

int32_t g_param_values[px4::params::PARAM_COUNT];
float   g_param_values_f[px4::params::PARAM_COUNT];
hrt_abstime g_hrt_elapsed = 0;

#define N_CANAIS 2

/* Driver de saída de teste: guarda o que o mixer entrega. */
class SaidaTeste : public OutputModuleInterface
{
public:
	SaidaTeste() : OutputModuleInterface("teste", px4::wq_configurations::hp_default) {}
	bool updateOutputs(float outputs[MAX_ACTUATORS], unsigned num_outputs, unsigned) override
	{
		n = num_outputs;

		for (unsigned i = 0; i < num_outputs && i < N_CANAIS; i++) { recebido[i] = outputs[i]; }

		return true;
	}
	float recebido[N_CANAIS] {};
	unsigned n{0};
};

static uint16_t faixa(uint16_t lo, uint16_t hi)
{
	uint16_t v = nondet_uint16();
	__VERIFIER_assume(v >= lo && v <= hi);
	return v;
}

/* parâmetros dos canais com as faixas do pwm_out/module.yaml */
static void configura(MixingOutput &m)
{
	for (int i = 0; i < N_CANAIS; i++) {
		m._disarmed_value[i] = faixa(800, 2200);
		m._min_value[i]      = faixa(800, 1400);
		m._center_value[i]   = faixa(800, 2200);
		m._max_value[i]      = faixa(1600, 2200);
		m._failsafe_value[i] = nondet_bool() ? (uint16_t)UINT16_MAX : faixa(800, 2200);
		m._function_assignment[i] = (OutputFunction)nondet_int();
		m._param_handles[i].center = nondet_bool() ? PARAM_INVALID : 0;
		m._functions[i] = nullptr;             /* valores vêm de outputs[] livres */
	}

	m._reverse_output_mask = nondet_uint16();
}

static int canal()
{
	int i = nondet_int();
	__VERIFIER_assume(i >= 0 && i < N_CANAIS);
	return i;
}

/* MX1 — output_limit_calc_single */
static void test_limite_canal(bool centro_na_faixa)
{
	SaidaTeste saida;
	MixingOutput m("PWM_MAIN", N_CANAIS, saida, MixingOutput::SchedulingPolicy::Disabled, false);
	configura(m);
	int i = canal();

	if (centro_na_faixa) {
		__VERIFIER_assume(m._center_value[i] >= m._min_value[i] && m._center_value[i] <= m._max_value[i]);
	}

	float v = nondet_float();
	float out = m.output_limit_calc_single(i, v);

	if (!std::isfinite(v)) {
		assert(out == (float)m._disarmed_value[i]);

	} else {
		assert(out >= (float)m._min_value[i] && out <= (float)m._max_value[i]);
	}
}

/* MX2, MX3, MX4 — limitAndUpdateOutputs em terminação / lockdown / kill */
static void test_terminacao(int qual)
{
	SaidaTeste saida;
	MixingOutput m("PWM_MAIN", N_CANAIS, saida, MixingOutput::SchedulingPolicy::Disabled, false);
	configura(m);

	m._armed.armed = nondet_bool();
	m._armed.prearmed = nondet_bool();
	m._armed.in_esc_calibration_mode = false;          /* o MixingOutput do teste não aceita calibração */

	if (qual == 4) {
		m._armed.lockdown = nondet_bool();
		m._armed.kill = !m._armed.lockdown || nondet_bool();
		m._armed.termination = nondet_bool();

	} else {
		m._armed.lockdown = false;
		m._armed.kill = false;
		m._armed.termination = true;
	}

	float outputs[MixingOutput::MAX_ACTUATORS];

	for (int k = 0; k < MixingOutput::MAX_ACTUATORS; k++) { outputs[k] = nondet_float(); }

	m.limitAndUpdateOutputs(outputs, false);

	assert(saida.n == N_CANAIS);
	int i = canal();
	float out = saida.recebido[i];

	if (qual == 2) {          /* MX2 */
		if (m._failsafe_value[i] != UINT16_MAX) {
			assert(out == (float)m._failsafe_value[i]);

		} else {
			assert(out == (float)m._disarmed_value[i]);   /* sem função: failsafe padrão NaN -> desarmado */
		}

	} else if (qual == 3) {   /* MX3 */
		assert(out >= (float)m._min_value[i] && out <= (float)m._max_value[i]);

	} else {                  /* MX4 */
		assert(out == (float)m._disarmed_value[i]);
	}
}

/* MX5 — output_limit_calc: estados OFF/RAMP/ON, tempo e pré-arme livres */
static void test_rampa()
{
	SaidaTeste saida;
	MixingOutput m("PWM_MAIN", N_CANAIS, saida, MixingOutput::SchedulingPolicy::Disabled, false);
	configura(m);

	for (int k = 0; k < N_CANAIS; k++) {
		__VERIFIER_assume(m._center_value[k] >= m._min_value[k] && m._center_value[k] <= m._max_value[k]);
	}

	int estado = nondet_int();
	__VERIFIER_assume(estado >= 0 && estado <= 2);
	m._output_state = (MixingOutput::OutputLimitState)estado;
	m._armed.armed = nondet_bool();
	m._armed.prearmed = nondet_bool();
	m._armed.in_esc_calibration_mode = nondet_bool();
	g_hrt_elapsed = nondet_uint64();

	float outputs[MixingOutput::MAX_ACTUATORS];

	for (int k = 0; k < MixingOutput::MAX_ACTUATORS; k++) { outputs[k] = nondet_float(); }

	m.output_limit_calc(nondet_bool(), N_CANAIS, outputs);

	int i = canal();
	float out = m._current_output_value[i];
	float d = (float)m._disarmed_value[i];
	float lo = d < (float)m._min_value[i] ? d : (float)m._min_value[i];
	float hi = d > (float)m._max_value[i] ? d : (float)m._max_value[i];
	assert(std::isfinite(out));
	assert(out >= lo && out <= hi);
}

/* Ponto de entrada: --function harness_main (mesmo padrão dos outros módulos). */
int harness_main()
{
#ifdef TEST_CHOICE
	int choice = TEST_CHOICE;                 /* rodar um teste por vez: -DTEST_CHOICE=n */
#else
	int choice = nondet_int();
	__VERIFIER_assume(choice >= 0 && choice <= 5);
#endif

	switch (choice) {
	case 0: test_limite_canal(false); break;
	case 1: test_limite_canal(true);  break;
	case 2: test_terminacao(2);       break;
	case 3: test_terminacao(3);       break;
	case 4: test_terminacao(4);       break;
	case 5: test_rampa();             break;
	}

	return 0;
}

int main() { return harness_main(); }
