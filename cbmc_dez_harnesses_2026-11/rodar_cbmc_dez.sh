#!/usr/bin/env bash
# CBMC nos dez harnesses sem violacao no ESBMC (pedido do orientador).
# Mesmo harness e mesmas pre-condicoes: so se troca __ESBMC_assume por __CPROVER_assume.
# Barometer e Battery sao analisados como C (o analisador C++ do CBMC 5.95.1 nao le 'a < 0 || a > 100'); os demais como C++.
# Opcoes equivalentes ao --overflow-check do ESBMC; sem unwinding assertions, como na 1a campanha,
# e uma segunda rodada COM unwinding assertions.
set -u
cd "$(dirname "$0")/.."
OUT=cbmc_dez_harnesses_2026-11/logs; mkdir -p "$OUT"; TMP=$(mktemp -d)
FL="--bounds-check --pointer-check --signed-overflow-check --unsigned-overflow-check"
{ date -u +%FT%TZ; uname -a; cbmc --version; } > "$OUT/ambiente.txt" 2>&1
LISTA='Barometer|resultados/Barometer/test_barometer.cpp|10|c
Battery|resultados/Battery_Monitor/test_battery.cpp|10|c
EKF2|resultados/EKF2_Covariance/test_ekf2_covariance.cpp|10|cpp
Expo|resultados/Expo_RC/test_expo_mathlib.cpp|10|cpp
Magnetometer|resultados/Magnetometer/test_vehicle_magnetometer.cpp|10|cpp
MAVLink|resultados/MAVLink/test_mavlink_parser.cpp|300|cpp
Navigator|resultados/Navigator_Mission/test_navigator_mission.cpp|15|cpp
PWM|resultados/PWM_Output/test_pwm_out.cpp|10|cpp
RC|resultados/RC_Update/test_rc_update.cpp|10|cpp
VTOL|resultados/VTOL/test_vtol_type.cpp|10|cpp'
echo "$LISTA" | while IFS='|' read -r nome arq k ext; do
  f="$TMP/$nome.$ext"
  grep -v 'extern void *__ESBMC_assume' "$arq" | sed 's/__ESBMC_assume(/__CPROVER_assume(/g' > "$f"
  for modo in sem com; do
    if [ $modo = sem ]; then UA="--no-unwinding-assertions"; else UA="--unwinding-assertions"; fi
    lg="$OUT/${nome}_${modo}_unwinding.log"
    { echo "# data: $(date -u +%FT%TZ)"; echo "# comando: cbmc $nome.$ext --unwind $k $UA $FL   (harness derivado de $arq)"; } > "$lg"
    t0=$(date +%s); timeout 900 cbmc "$f" --unwind "$k" $UA $FL >> "$lg" 2>&1; echo "# exit: $?  tempo_de_parede: $(( $(date +%s)-t0 ))s" >> "$lg"
    v=$(grep -aoE 'VERIFICATION (SUCCESSFUL|FAILED)' "$lg" | tail -1)
    n=$(grep -ac '^\[.*FAILURE' "$lg"); pf=$(grep -aE '^\*\* [0-9]+ of [0-9]+ failed' "$lg" | tail -1)
    printf '%-13s k=%-3s %-4s unwinding | %-24s | %s | %s\n' "$nome" "$k" "$modo" "${v:-SEM VEREDITO}" "${pf:-}" "$(grep -a '^# exit' "$lg" | sed 's/# //')" | tee -a "$OUT/RESUMO.txt"
  done
done
