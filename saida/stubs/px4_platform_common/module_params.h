/*
 * STUB de module_params.h — parâmetros tipados (ParamInt/ParamFloat).
 * O valor de cada parâmetro é escolhido pelo harness (variáveis globais
 * g_param_<NOME>), para que as pré-condições venham do module.yaml.
 */
#pragma once
#include "px4_common_stub.h"

namespace px4 { namespace params {
enum params_t { RBCLW_ADDRESS, RBCLW_COUNTS_REV, MC_AIRMODE, THR_MDL_FAC, PARAM_COUNT };
} }

extern int32_t g_param_values[px4::params::PARAM_COUNT];

template<px4::params::params_t P>
class ParamInt
{
public:
	int32_t get() const { return g_param_values[P]; }
	int32_t &reference() { return g_param_values[P]; }
};

extern float g_param_values_f[px4::params::PARAM_COUNT];

template<px4::params::params_t P>
class ParamFloat
{
public:
	float get() const { return g_param_values_f[P]; }
	float &reference() { return g_param_values_f[P]; }
};

/* STUB_MODULEPARAMS_SEM_VIRTUAL: ModuleParams não polimórfica. Contorno de um erro do
 * ESBMC 8.5 ("bad optional access" ao construir classe polimórfica com membro vetor
 * inicializado com {}, como o MixingOutput); ver saida/ferramenta/esbmc_crash_vetor_polimorfico.cpp.
 * O harness que usa isto também define "override" como vazio. */
class ModuleParams
{
public:
	ModuleParams() = default;
	explicit ModuleParams(ModuleParams *) {}
#ifdef STUB_MODULEPARAMS_SEM_VIRTUAL
	void updateParams() {}
#else
	virtual ~ModuleParams() = default;
	virtual void updateParams() {}
#endif
};

/* DEFINE_PARAMETERS((Tipo) nome, ...) -> "Tipo nome; ..." (até 8 parâmetros) */
#define _DP_EAT(...) __VA_ARGS__
#define _DP_DECL(x) _DP_EAT x;
#define _DP_1(a) _DP_DECL(a)
#define _DP_2(a, ...) _DP_DECL(a) _DP_1(__VA_ARGS__)
#define _DP_3(a, ...) _DP_DECL(a) _DP_2(__VA_ARGS__)
#define _DP_4(a, ...) _DP_DECL(a) _DP_3(__VA_ARGS__)
#define _DP_5(a, ...) _DP_DECL(a) _DP_4(__VA_ARGS__)
#define _DP_6(a, ...) _DP_DECL(a) _DP_5(__VA_ARGS__)
#define _DP_7(a, ...) _DP_DECL(a) _DP_6(__VA_ARGS__)
#define _DP_8(a, ...) _DP_DECL(a) _DP_7(__VA_ARGS__)
#define _DP_N(_1, _2, _3, _4, _5, _6, _7, _8, N, ...) N
#define DEFINE_PARAMETERS(...) _DP_N(__VA_ARGS__, _DP_8, _DP_7, _DP_6, _DP_5, _DP_4, _DP_3, _DP_2, _DP_1, 0)(__VA_ARGS__)
