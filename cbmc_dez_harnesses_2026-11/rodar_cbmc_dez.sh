#!/usr/bin/env bash
# CBMC nos dez harnesses sem violacao no ESBMC (pedido do orientador).
# Mesmo harness e mesmas pre-condicoes: so se troca __ESBMC_assume por __CPROVER_assume.
# Navigator usa isfinite(): o CBMC 5.95.1 nao tem corpo para __builtin_isfinite (retorna valor arbitrario), entao ele e redefinido com isnan/isinf.
# O leitor C++ do CBMC 5.95.1 tem limitacoes: nao le 'a < 0 || a > 100' e nao le <cmath> com o glibc 2.39 ('_Float32').
# Por isso, oito harnesses (Barometer, Battery, EKF2, Magnetometer, Navigator, PWM, RC, VTOL) sao analisados como C
# (cabecalhos <cX> trocados por <X.h>); Expo (template) e MAVLink (class) sao C++ de fato e usam --cpp11; no Expo,
# <cmath> e substituido por __builtin_isnan/__builtin_isinf (modelagem registrada no cabecalho de cada log).
# Opcoes equivalentes ao --overflow-check do ESBMC; sem unwinding assertions, como na 1a campanha,
# e uma segunda rodada COM unwinding assertions.
set -u
cd "$(dirname "$0")/.."
OUT=cbmc_dez_harnesses_2026-11/logs; mkdir -p "$OUT"; TMP=$(mktemp -d)
FL="--bounds-check --pointer-check --signed-overflow-check --unsigned-overflow-check"
{ date -u +%FT%TZ; uname -a; cbmc --version; } > "$OUT/ambiente.txt" 2>&1
LISTA='Barometer|resultados/Barometer/test_barometer.cpp|10|c
Battery|resultados/Battery_Monitor/test_battery.cpp|10|c
EKF2|resultados/EKF2_Covariance/test_ekf2_covariance.cpp|10|c
Expo|resultados/Expo_RC/test_expo_mathlib.cpp|10|cpp
Magnetometer|resultados/Magnetometer/test_vehicle_magnetometer.cpp|10|c
MAVLink|resultados/MAVLink/test_mavlink_parser.cpp|300|cpp
Navigator|resultados/Navigator_Mission/test_navigator_mission.cpp|15|c
PWM|resultados/PWM_Output/test_pwm_out.cpp|10|c
RC|resultados/RC_Update/test_rc_update.cpp|10|c
VTOL|resultados/VTOL/test_vtol_type.cpp|10|c'
# ONLY="EKF2|Expo" roda so esses (padrao: todos).
echo "$LISTA" | while IFS='|' read -r nome arq k ext; do
  if [ -n "${ONLY:-}" ] && ! echo "|$ONLY|" | grep -q "|$nome|"; then continue; fi
  f="$TMP/$nome.$ext"
  if [ $ext = c ]; then
    grep -v 'extern void *__ESBMC_assume' "$arq" | sed 's/__ESBMC_assume(/__CPROVER_assume(/g; s/#include <c\([a-z]*\)>/#include <\1.h>/; s|#include <math.h>|#include <math.h>\n#undef isfinite\n#define isfinite(x) (!isnan(x) \&\& !isinf(x))|' > "$f"; STD=""
  else
    grep -v 'extern void *__ESBMC_assume' "$arq" | sed 's/__ESBMC_assume(/__CPROVER_assume(/g; s/#include <cmath>/#define isnan(x) __builtin_isnan(x)\n#define isinf(x) __builtin_isinf(x)/' > "$f"; STD="--cpp11"
  fi
  for modo in sem com; do
    if [ $modo = sem ]; then UA="--no-unwinding-assertions"; else UA="--unwinding-assertions"; fi
    lg="$OUT/${nome}_${modo}_unwinding.log"
    { echo "# data: $(date -u +%FT%TZ)"; echo "# comando: cbmc $nome.$ext $STD --unwind $k $UA $FL   (harness derivado de $arq; leitura como $ext)"; } > "$lg"
    t0=$(date +%s); timeout 900 cbmc "$f" $STD --unwind "$k" $UA $FL >> "$lg" 2>&1; echo "# exit: $?  tempo_de_parede: $(( $(date +%s)-t0 ))s" >> "$lg"
    v=$(grep -aoE 'VERIFICATION (SUCCESSFUL|FAILED)' "$lg" | tail -1)
    n=$(grep -ac '^\[.*FAILURE' "$lg"); pf=$(grep -aE '^\*\* [0-9]+ of [0-9]+ failed' "$lg" | tail -1)
    printf '%-13s k=%-3s %-4s unwinding | %-24s | %s | %s\n' "$nome" "$k" "$modo" "${v:-SEM VEREDITO}" "${pf:-}" "$(grep -a '^# exit' "$lg" | sed 's/# //')" | tee -a "$OUT/RESUMO.txt"
  done
done
