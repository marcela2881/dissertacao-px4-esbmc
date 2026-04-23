/**
 * @file test_expo_mathlib_v4.cpp
 * @author Dissertação Mestrado - Verificação Formal PX4 v1.16
 *
 * OBJETIVO: Verificação formal de math::expo() do PX4
 * FUNÇÃO TESTADA: expo() - src/lib/mathlib/math/Functions.hpp
 * MÉTODO: Bounded Model Checking com ESBMC
 * VERSÃO: v4 - Foco em safety properties (sem comparações de precisão)
 */

#include <assert.h>
#include <cmath>

extern int nondet_int();
extern float nondet_float();
extern void __ESBMC_assume(int condition);

// ================== FUNÇÃO REAL DO PX4 ==================
template<typename T>
T constrain(const T &val, const T &min_val, const T &max_val)
{
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

template<typename T>
const T expo(const T &value, const T &e)
{
    T x  = constrain(value, (T)-1, (T)1);
    T ec = constrain(e,     (T) 0, (T)1);
    return (1 - ec) * x + ec * x * x * x;
}

// ================== TESTES ==================

/**
 * TESTE 1: value simbólico, e=0 (linear)
 * Verifica: resultado finito e no range [-1,1]
 */
void test_expo_linear_safe() {
    float value = nondet_float();
    __ESBMC_assume(value >= -1.0f && value <= 1.0f);
    __ESBMC_assume(!isnan(value) && !isinf(value));

    float result = expo(value, 0.0f);

    assert(!isnan(result));
    assert(!isinf(result));
    assert(result >= -1.0f && result <= 1.0f);
}

/**
 * TESTE 2: value simbólico, e=1 (cúbico)
 * Verifica: resultado finito e no range [-1,1]
 */
void test_expo_cubic_safe() {
    float value = nondet_float();
    __ESBMC_assume(value >= -1.0f && value <= 1.0f);
    __ESBMC_assume(!isnan(value) && !isinf(value));

    float result = expo(value, 1.0f);

    assert(!isnan(result));
    assert(!isinf(result));
    assert(result >= -1.0f && result <= 1.0f);
}

/**
 * TESTE 3: value=0, e simbólico
 * Verifica: expo(0,e) = 0 sempre
 */
void test_expo_zero_input() {
    float e = nondet_float();
    __ESBMC_assume(e >= 0.0f && e <= 1.0f);
    __ESBMC_assume(!isnan(e) && !isinf(e));

    float result = expo(0.0f, e);

    assert(!isnan(result));
    assert(!isinf(result));
    assert(result >= -1e-5f && result <= 1e-5f);
}

/**
 * TESTE 4: value=1, e simbólico
 * Verifica: resultado sempre positivo
 */
void test_expo_max_input() {
    float e = nondet_float();
    __ESBMC_assume(e >= 0.0f && e <= 1.0f);
    __ESBMC_assume(!isnan(e) && !isinf(e));

    float result = expo(1.0f, e);

    assert(!isnan(result));
    assert(!isinf(result));
    assert(result >= 0.0f && result <= 1.0f);
}

/**
 * TESTE 5: value=-1, e simbólico
 * Verifica: resultado sempre negativo
 */
void test_expo_min_input() {
    float e = nondet_float();
    __ESBMC_assume(e >= 0.0f && e <= 1.0f);
    __ESBMC_assume(!isnan(e) && !isinf(e));

    float result = expo(-1.0f, e);

    assert(!isnan(result));
    assert(!isinf(result));
    assert(result >= -1.0f && result <= 0.0f);
}

// ================== MAIN ==================
int main() {
    int test_choice = nondet_int();
    __ESBMC_assume(test_choice >= 0 && test_choice < 5);

    switch(test_choice) {
        case 0: test_expo_linear_safe(); break;
        case 1: test_expo_cubic_safe();  break;
        case 2: test_expo_zero_input();  break;
        case 3: test_expo_max_input();   break;
        case 4: test_expo_min_input();   break;
    }

    return 0;
}
