/*
 * STUB comum — itens de plataforma do PX4 usados pelos drivers de saída
 * (constantes, literais de tempo, parâmetros, log, matemática).
 */
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <climits>
#include <unistd.h>
/* <fcntl.h> NÃO é incluído: no Linux ele traz struct iovec (bits/fcntl-linux.h),
 * que o ESBMC 8.5 já define internamente -> "redefinition of 'iovec'".
 * Só o que os drivers usam (valores do Linux/x86-64): */
#ifndef O_RDWR
#define O_RDWR   02
#endif
#ifndef O_NOCTTY
#define O_NOCTTY 0400
#endif
extern "C" int open(const char *path, int flags, ...);
#include <err.h>
#include "../px4_log.h"

#ifndef OK
#define OK 0
#endif
#ifndef ERROR
#define ERROR (-1)
#endif
#ifndef MODULE_NAME
#define MODULE_NAME "modulo"
#endif
#ifndef M_TWOPI_F
#define M_TWOPI_F 6.28318530717958647692f
#endif
#ifndef __EXPORT
#define __EXPORT
#endif

typedef uint64_t hrt_abstime;
inline hrt_abstime hrt_absolute_time() { return 0; }   /* tempo não modelado */

namespace time_literals
{
constexpr hrt_abstime operator ""_ms(unsigned long long v) { return v * 1000ULL; }
constexpr hrt_abstime operator ""_s(unsigned long long v)  { return v * 1000000ULL; }
constexpr hrt_abstime operator ""_us(unsigned long long v) { return v; }
}
using namespace time_literals;

/* STUB_MATHLIB_ORIGINAL: o harness usa o src/lib/mathlib ORIGINAL (que já define math::constrain) */
#ifndef STUB_MATHLIB_ORIGINAL
namespace math
{
template<typename T> constexpr T constrain(T v, T lo, T hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }
}
#endif

/* parâmetros por nome (param_find/param_get): devolvem 0 (não usados nas propriedades) */
typedef int param_t;
#define PARAM_INVALID (-1)
inline param_t param_find(const char *) { return 0; }
inline int param_get(param_t, void *val) { if (val) { *(int32_t *)val = 0; } return 0; }

namespace px4 { namespace wq_configurations { struct wq_config_t { int id; }; static constexpr wq_config_t hp_default{0}; } }
