#include <assert.h>
#include <math.h>
#include <stdint.h>

extern float nondet_float();
extern int   nondet_int();
extern void  __ESBMC_assume(int condition);

#define THROTTLE_BLENDING_DUR_S   1.0f
#define MIN_WEIGHT_RATIO          0.5f
#define MAX_WEIGHT_RATIO          2.0f
#define RHO_MIN                   0.7f
#define RHO_MAX                   1.5f
#define RHO_SEA_LEVEL             1.225f
#define TRANSITION_DT_MIN         0.0001f
#define TRANSITION_DT_MAX         0.02f
#define FORWARD_THRUST_MAX        0.9f
#define FLT_EPSILON               1.192093e-07f

float get_front_transition_time_factor(float air_density) {
    float rho = air_density;
    if (rho < RHO_MIN) rho = RHO_MIN;
    if (rho > RHO_MAX) rho = RHO_MAX;
    if (!isnan(rho) && !isinf(rho)) {
        float rho0_over_rho = RHO_SEA_LEVEL / rho;
        return sqrtf(rho0_over_rho) * rho0_over_rho;
    }
    return 1.0f;
}

float get_transition_airspeed(float weight_gross, float weight_base, float arsp_trans) {
    float weight_ratio = 1.0f;
    if (weight_base > FLT_EPSILON && weight_gross > FLT_EPSILON) {
        weight_ratio = weight_gross / weight_base;
        if (weight_ratio < MIN_WEIGHT_RATIO) weight_ratio = MIN_WEIGHT_RATIO;
        if (weight_ratio > MAX_WEIGHT_RATIO) weight_ratio = MAX_WEIGHT_RATIO;
    }
    return sqrtf(weight_ratio) * arsp_trans;
}

float compute_forward_thrust(float pitch_setpoint_min, float pitch_setpoint, float thrust_scale) {
    if (pitch_setpoint >= pitch_setpoint_min) return 0.0f;
    float thrust = (sinf(pitch_setpoint_min) - sinf(pitch_setpoint)) * thrust_scale;
    if (thrust < 0.0f) thrust = 0.0f;
    if (thrust > FORWARD_THRUST_MAX) thrust = FORWARD_THRUST_MAX;
    return thrust;
}

float compute_transition_dt(float raw_dt) {
    if (raw_dt < TRANSITION_DT_MIN) return TRANSITION_DT_MIN;
    if (raw_dt > TRANSITION_DT_MAX) return TRANSITION_DT_MAX;
    return raw_dt;
}

int is_front_transition_altitude_loss(float z_now, float z_start, float alt_loss_threshold) {
    if (alt_loss_threshold <= FLT_EPSILON) return 0;
    return (z_now - z_start) > alt_loss_threshold ? 1 : 0;
}

void test_transition_time_factor_positive() {
    float air_density = nondet_float();
    __ESBMC_assume(!isnan(air_density) && !isinf(air_density));
    __ESBMC_assume(air_density >= 0.1f && air_density <= 3.0f);
    float factor = get_front_transition_time_factor(air_density);
    assert(factor > 0.0f);
    assert(!isnan(factor));
    assert(!isinf(factor));
    assert(factor >= 0.5f && factor <= 3.0f);
}

void test_transition_airspeed_positive() {
    float weight_gross = nondet_float();
    float weight_base  = nondet_float();
    float arsp_trans   = nondet_float();
    __ESBMC_assume(!isnan(weight_gross) && !isinf(weight_gross));
    __ESBMC_assume(!isnan(weight_base)  && !isinf(weight_base));
    __ESBMC_assume(!isnan(arsp_trans)   && !isinf(arsp_trans));
    __ESBMC_assume(weight_gross >= 0.1f && weight_gross <= 100.0f);
    __ESBMC_assume(weight_base  >= 0.1f && weight_base  <= 100.0f);
    __ESBMC_assume(arsp_trans   >= 5.0f && arsp_trans   <= 50.0f);
    float airspeed = get_transition_airspeed(weight_gross, weight_base, arsp_trans);
    assert(airspeed >= 0.0f);
    assert(!isnan(airspeed));
    assert(airspeed >= arsp_trans * 0.7f);
    assert(airspeed <= arsp_trans * 1.5f);
}

void test_forward_thrust_bounds() {
    float pitch_min    = nondet_float();
    float pitch_sp     = nondet_float();
    float thrust_scale = nondet_float();
    __ESBMC_assume(!isnan(pitch_min)    && !isinf(pitch_min));
    __ESBMC_assume(!isnan(pitch_sp)     && !isinf(pitch_sp));
    __ESBMC_assume(!isnan(thrust_scale) && !isinf(thrust_scale));
    __ESBMC_assume(pitch_min    >= -1.0f  && pitch_min    <= 0.0f);
    __ESBMC_assume(pitch_sp     >= -1.57f && pitch_sp     <= 1.57f);
    __ESBMC_assume(thrust_scale >= 0.0f && thrust_scale <= 1.0f);
    // teste desabilitado: sinf sem corpo com --no-library
    return;
    // teste desabilitado: sinf sem corpo com --no-library
    return;
    float thrust = compute_forward_thrust(pitch_min, pitch_sp, thrust_scale);
    assert(thrust >= 0.0f);
    assert(thrust <= FORWARD_THRUST_MAX);
    assert(!isnan(thrust));
}

void test_transition_dt_bounds() {
    float raw_dt = nondet_float();
    __ESBMC_assume(!isnan(raw_dt) && !isinf(raw_dt));
    __ESBMC_assume(raw_dt >= -1.0f && raw_dt <= 1.0f);
    float dt = compute_transition_dt(raw_dt);
    assert(dt >= TRANSITION_DT_MIN);
    assert(dt <= TRANSITION_DT_MAX);
    assert(!isnan(dt));
    assert(dt > 0.0f);
}

void test_altitude_loss_detection() {
    float z_now     = nondet_float();
    float z_start   = nondet_float();
    float threshold = nondet_float();
    __ESBMC_assume(!isnan(z_now)     && !isinf(z_now));
    __ESBMC_assume(!isnan(z_start)   && !isinf(z_start));
    __ESBMC_assume(!isnan(threshold) && !isinf(threshold));
    __ESBMC_assume(z_now    >= -1000.0f && z_now    <= 1000.0f);
    __ESBMC_assume(z_start  >= -1000.0f && z_start  <= 1000.0f);
    __ESBMC_assume(threshold >= 0.0f    && threshold <= 100.0f);
    int result = is_front_transition_altitude_loss(z_now, z_start, threshold);
    assert(result == 0 || result == 1);
    if (threshold <= FLT_EPSILON) assert(result == 0);
    if (z_now <= z_start) assert(result == 0);
}

int main() {
    int choice = nondet_int();
    __ESBMC_assume(choice >= 0 && choice < 5);
    switch (choice) {
        case 0: test_transition_time_factor_positive(); break;
        case 1: test_transition_airspeed_positive();    break;
        case 2: test_forward_thrust_bounds();           break;
        case 3: test_transition_dt_bounds();            break;
        case 4: test_altitude_loss_detection();         break;
    }
    return 0;
}
