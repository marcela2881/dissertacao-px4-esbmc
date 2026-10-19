/*
 * Harness de verificação — alocação de controle do PX4 (src/lib/control_allocation)
 *
 * O código NÃO é copiado: este arquivo inclui os .cpp ORIGINAIS do PX4
 * (ControlAllocation, ControlAllocationPseudoInverse e
 * ControlAllocationSequentialDesaturation) e a biblioteca matrix original. Só os
 * parâmetros (ModuleParams) e o tópico control_allocator_status vêm dos stubs.
 *
 * Recorte: a conversão do pedido de controle (torques e empuxo) em comandos dos
 * atuadores, entre 0 e 1, que o control_allocator entrega ao mixer.
 *
 * PRÉ-CONDIÇÕES (origem anotada):
 *   - número de atuadores em [1, 16] (ActuatorEffectiveness::NUM_ACTUATORS);
 *   - limites e trim por atuador em [-1, 1]: o ControlAllocator só usa min -1 ou 0 e
 *     max 1 (ControlAllocator.cpp, linhas 513-536); a faixa inclui min > max, que a
 *     classe trata (clip usa o trim);
 *   - valor do atuador antes do limite: |v| <= 100 (hipótese: saída da alocação para
 *     pedidos normalizados; evita estouro de float artificial em v - anterior);
 *   - dt em [0,0002; 0,02] s (ControlAllocator.cpp, linha 372, math::constrain);
 *   - taxa de variação (CA_R*_SLEW / CA_SV*_SLEW) em (FLT_EPSILON, 10] (module.yaml;
 *     com 0 a função não limita);
 *   - CA4: quadricóptero em X com matriz de efetividade fixa (quatro motores,
 *     colunas roll/pitch/yaw/empuxo), pedido de controle livre: torques em [-1, 1] e
 *     empuxo z em [-1, 0] (faixa normalizada que o controlador de atitude publica);
 *     MC_AIRMODE em {0, 1, 2} (mixer_module/params.yaml).
 *
 * PROPRIEDADES (especificação: comentários do próprio código e definição das funções):
 *   CA1 (T0) clipActuatorSetpoint: entrada finita e min <= max -> saída em [min, max];
 *       min > max -> saída = trim ("clip").
 *   CA2 (T1) applySlewRateLimit, comportamento documentado no comentário da função:
 *       "between 0 and NaN: immediately match input";
 *       "nonzero to NaN: sink to zero with slew rate, then replace zero by NaN";
 *       "NaN to nonzero: replace NaN by zero, then rise with slew rate to input".
 *   CA3 (T2) normalizeActuatorSetpoint: limites do ControlAllocator (min -1 ou 0, max 1) e
 *       valor em [min, max] -> resultado em [0, 1].
 *   CA4 (T3) ControlAllocationSequentialDesaturation::allocate (quadricóptero X):
 *       pedido finito -> todos os comandos finitos (sem NaN nem infinito).
 */

#define STUB_MATHLIB_ORIGINAL 1             /* math:: e matrix:: vêm do src/lib original */
#define STUB_MODULEPARAMS_SEM_VIRTUAL 1     /* mesmo contorno do ESBMC 8.5 usado no mixer */
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>
#include <cfloat>
#include <cassert>

extern "C" {
int   nondet_int();
float nondet_float();
void  __VERIFIER_assume(int);   /* aceito pelo ESBMC e pelo CBMC */
}

#include <px4_platform_common/px4_common_stub.h>
#include <mathlib/math/Functions.hpp>
#include <matrix/matrix/math.hpp>

#define override
#define private public
#define protected public
#include "control_allocation/ControlAllocation.cpp"                       /* ORIGINAL */
#include "control_allocation/ControlAllocationPseudoInverse.cpp"          /* ORIGINAL */
#include "control_allocation/ControlAllocationSequentialDesaturation.cpp" /* ORIGINAL */
#undef private
#undef protected
#undef override

int32_t g_param_values[px4::params::PARAM_COUNT];
float   g_param_values_f[px4::params::PARAM_COUNT];

static constexpr int NA = ControlAllocation::NUM_ACTUATORS;

static float finito()
{
	float v = nondet_float();
	__VERIFIER_assume(std::isfinite(v));
	return v;
}

static float faixa(float lo, float hi)
{
	float v = nondet_float();
	__VERIFIER_assume(v >= lo && v <= hi);   /* NaN não passa nas comparações */
	return v;
}

static int num_atuadores(ControlAllocation &ca)
{
	int n = nondet_int();
	__VERIFIER_assume(n >= 1 && n <= NA);
	ca._num_actuators = n;
	return n;
}

/* CA1 — clipActuatorSetpoint */
static void test_clip()
{
	ControlAllocationPseudoInverse ca;
	int n = num_atuadores(ca);
	ControlAllocation::ActuatorVector v;

	for (int i = 0; i < NA; i++) {
		ca._actuator_min(i) = faixa(-1.f, 1.f);
		ca._actuator_max(i) = faixa(-1.f, 1.f);
		ca._actuator_trim(i) = faixa(-1.f, 1.f);
		v(i) = finito();                          /* clip não faz conta: qualquer valor finito */
	}

	ControlAllocation::ActuatorVector entrada = v;
	ca.clipActuatorSetpoint(v);

	int i = nondet_int();
	__VERIFIER_assume(i >= 0 && i < n);

	if (ca._actuator_max(i) < ca._actuator_min(i)) {
		assert(v(i) == ca._actuator_trim(i));

	} else {
		assert(v(i) >= ca._actuator_min(i) && v(i) <= ca._actuator_max(i));
		(void)entrada;
	}
}

/* CA2 — applySlewRateLimit (um atuador; o laço trata cada atuador da mesma forma) */
static void test_slew()
{
	ControlAllocationPseudoInverse ca;
	ca._num_actuators = 1;

	float min = faixa(-1.f, 1.f), max = faixa(-1.f, 1.f);
	__VERIFIER_assume(min < max);
	float slew = faixa(0.f, 10.f);
	__VERIFIER_assume(slew > FLT_EPSILON);
	float dt = faixa(0.0002f, 0.02f);

	ca._actuator_min(0) = min;
	ca._actuator_max(0) = max;
	ca._actuator_slew_rate_limit(0) = slew;

	int caso = nondet_int();
	__VERIFIER_assume(caso >= 0 && caso <= 2);
	float anterior, entrada;

	if (caso == 0) {          /* entre 0 e NaN, nos dois sentidos */
		anterior = (nondet_int() & 1) ? 0.f : NAN;
		entrada  = (nondet_int() & 1) ? 0.f : NAN;

	} else if (caso == 1) {   /* não nulo -> NaN */
		anterior = faixa(-100.f, 100.f);
		__VERIFIER_assume(anterior != 0.f);
		entrada = NAN;

	} else {                  /* NaN -> não nulo */
		anterior = NAN;
		entrada = faixa(-100.f, 100.f);
		__VERIFIER_assume(entrada != 0.f);
	}

	ca._prev_actuator_sp(0) = anterior;
	ca._actuator_sp(0) = entrada;
	ca.applySlewRateLimit(dt);
	float saida = ca._actuator_sp(0);

	if (caso == 0) {
		/* "immediately match input" */
		assert(std::isnan(entrada) ? std::isnan(saida) : saida == entrada);

	} else if (caso == 1) {
		/* "sink to zero with slew rate, then replace zero by NaN" */
		assert(std::isnan(saida) || std::fabs(saida) <= std::fabs(anterior));

	} else {
		/* "replace NaN by zero, then rise with slew rate to input" */
		assert(std::isfinite(saida));
		assert(std::fabs(saida) <= std::fabs(entrada));
		assert((saida >= 0.f) == (entrada > 0.f) || saida == 0.f);
	}
}

/* CA3 — normalizeActuatorSetpoint */
static void test_normaliza()
{
	ControlAllocationPseudoInverse ca;
	int n = num_atuadores(ca);
	ControlAllocation::ActuatorVector v;

	for (int i = 0; i < NA; i++) {
		/* limites exatamente como o ControlAllocator configura: min -1 (reversível/servo)
		 * ou 0 (motor), max 1 (ControlAllocator.cpp, linhas 513-536). Com limites
		 * quaisquer, a divisão (v - min) / (max - min) estoura quando max - min é
		 * subnormal, e com min == max o ramo "else" divide por zero; nenhum dos dois
		 * ocorre no PX4. */
		ca._actuator_min(i) = (nondet_int() & 1) ? -1.f : 0.f;
		ca._actuator_max(i) = 1.f;
		ca._actuator_trim(i) = 0.f;
		v(i) = faixa(-100.f, 100.f);             /* mesma hipótese |v| <= 100 */
	}

	int i = nondet_int();
	__VERIFIER_assume(i >= 0 && i < n);
	__VERIFIER_assume(v(i) >= ca._actuator_min(i) && v(i) <= ca._actuator_max(i));

	ControlAllocation::ActuatorVector r = ca.normalizeActuatorSetpoint(v);
	assert(r(i) >= 0.f && r(i) <= 1.f);
}

/* CA4 — alocação com dessaturação sequencial, quadricóptero em X */
static void test_alocacao()
{
	ControlAllocationSequentialDesaturation ca;

	int airmode = nondet_int();
	__VERIFIER_assume(airmode >= 0 && airmode <= 2);
	g_param_values[px4::params::MC_AIRMODE] = airmode;

	/* colunas: motores 1..4; linhas: roll, pitch, yaw, empuxo x, y, z */
	matrix::Matrix<float, ControlAllocation::NUM_AXES, NA> efetividade;
	const float roll[4]  = {-1.f,  1.f,  1.f, -1.f};
	const float pitch[4] = { 1.f, -1.f,  1.f, -1.f};
	const float yaw[4]   = { 1.f,  1.f, -1.f, -1.f};

	for (int m = 0; m < 4; m++) {
		efetividade(0, m) = roll[m];
		efetividade(1, m) = pitch[m];
		efetividade(2, m) = yaw[m];
		efetividade(5, m) = -1.f;
	}

	ControlAllocation::ActuatorVector zero;
	ControlAllocation::ActuatorVector um;
	um.setAll(1.f);
	ca.setActuatorMin(zero);
	ca.setActuatorMax(um);
	ca.setEffectivenessMatrix(efetividade, zero, zero, 4, true);

	matrix::Vector<float, ControlAllocation::NUM_AXES> pedido;

	for (int k = 0; k < 3; k++) {
		pedido(k) = finito();
		__VERIFIER_assume(pedido(k) >= -1.f && pedido(k) <= 1.f);
	}

	pedido(5) = finito();
	__VERIFIER_assume(pedido(5) >= -1.f && pedido(5) <= 0.f);

	ca.setControlSetpoint(pedido);
	ca.allocate();

	for (int m = 0; m < 4; m++) {
		assert(std::isfinite(ca.getActuatorSetpoint()(m)));
	}
}

/* Ponto de entrada: --function harness_main (mesmo padrão dos outros módulos). */
int harness_main()
{
#ifdef TEST_CHOICE
	int choice = TEST_CHOICE;                 /* rodar um teste por vez: -DTEST_CHOICE=n */
#else
	int choice = nondet_int();
	__VERIFIER_assume(choice >= 0 && choice <= 3);
#endif

	switch (choice) {
	case 0: test_clip();      break;
	case 1: test_slew();      break;
	case 2: test_normaliza(); break;
	case 3: test_alocacao();  break;
	}

	return 0;
}

int main() { return harness_main(); }
