/*
 * Prelúdio usado SÓ pelo CBMC nos harnesses C++ (o ESBMC não o usa).
 * O comparar.sh gera, para cada teste, um arquivo com
 *     #include "prelude_cbmc.h"
 *     #include "<harness original>"
 * de modo que o harness entra sem nenhuma alteração (mesma técnica usada para
 * incluir os .cpp originais do PX4).
 *
 * Motivo (PC do Bruno, Ubuntu 24.04: glibc 2.39, libstdc++ 13; CBMC 5.95.1 e
 * também 6.11.0): o frontend C++ do CBMC não aceita os cabeçalhos de
 * matemática do sistema:
 *   1. "symbol '_Float32' is unknown" (bits/mathcalls.h): com GCC >= 13 em C++
 *      a glibc supõe que o compilador conhece _Float32/_Float64/...; definir os
 *      tipos com -D só troca o erro por "__iseqsig_type defined previously"
 *      (especialização duplicada em math.h). Com __GNUC__ = 12 a glibc declara
 *      esses tipos ela mesma e não gera as especializações.
 *   2. "__builtin_fabsf128 is unknown" (bits/std_abs.h, abs(__float128),
 *      não usado pelos harnesses).
 *   3. Falha interna do CBMC "__CPROVER_isgreaterd was not found" (std::isgreater
 *      e afins em <cmath>): as macros abaixo dão o mesmo resultado lógico
 *      (falso se algum operando é NaN); só não modelam a flag FE_INVALID,
 *      que nenhum harness consulta.
 *   3b. __builtin_isfinite (usado por std::isfinite) não tem modelo no frontend
 *      C++ do CBMC: vira função sem corpo, com retorno ARBITRÁRIO. Achado no
 *      contraexemplo do Navigator_Mission (PC do Bruno, 19/10/2026):
 *      isfinite(6553.6f) devolveu falso. (x - x == 0) é verdadeiro só para x
 *      finito (infinito ou NaN dão NaN). isnan/isinf foram conferidos e estão
 *      corretos no CBMC.
 *   4. Harnesses que usam isnan/isinf/isfinite sem "std::" (o ESBMC aceita).
 *   5. __ESBMC_assume (função do ESBMC) = __CPROVER_assume (mesmo significado:
 *      pré-condição). Substitui o antigo shim_esbmc_assume.cpp, que não servia
 *      para harnesses C++ que não declaram a função.
 */
#ifndef PRELUDE_CBMC_H
#define PRELUDE_CBMC_H

#if defined(__GNUC__) && __GNUC__ >= 13
#undef __GNUC__
#define __GNUC__ 12
#endif

#define __builtin_fabsf128 __builtin_fabsl
#define __builtin_isgreater(x, y)      ((x) > (y))
#define __builtin_isgreaterequal(x, y) ((x) >= (y))
#define __builtin_isless(x, y)         ((x) < (y))
#define __builtin_islessequal(x, y)    ((x) <= (y))
#define __builtin_islessgreater(x, y)  ((x) < (y) || (x) > (y))
#define __builtin_isunordered(x, y)    ((x) != (x) || (y) != (y))
#define __builtin_isfinite(x)          ((x) - (x) == 0)

#include <cmath>
#include <math.h>
using std::isnan;
using std::isinf;
using std::isfinite;

inline void __ESBMC_assume(int condition) { __CPROVER_assume(condition != 0); }

#endif
