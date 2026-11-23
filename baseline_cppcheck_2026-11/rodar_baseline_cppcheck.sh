#!/usr/bin/env bash
# Baseline com analisador estatico (cppcheck) sobre os 12 modulos, a pedido do orientador.
# (1) sobre o CODIGO REAL do PX4 de cada modulo;  (2) sobre o HARNESS que o ESBMC verificou.
# Uso: bash rodar_baseline_cppcheck.sh CAMINHO_DA_ARVORE_DO_PX4   (pasta que contem src/)
# Arvore usada: commit 8be2590c15. cppcheck sem as dependencias do build: limite declarado no texto.
set -u
PX4="${1:?informe a arvore do PX4}"; cd "$(dirname "$0")/.."; REPO=$(pwd)
OUT=baseline_cppcheck_2026-11/logs; mkdir -p "$OUT"
{ date -u +%FT%TZ; uname -a; cppcheck --version; } > "$OUT/ambiente.txt"
FLAGS="--enable=warning,performance,portability --inconclusive --inline-suppr --suppress=missingInclude --suppress=missingIncludeSystem --suppress=unmatchedSuppression -q --max-configs=2 -j2 --template={file}:{line}:{severity}:{id}:{message}"
MODS='GPS|drivers/gps
IMU_BMI088|drivers/imu/bosch/bmi088
MAVLink|modules/mavlink
Expo_RC|lib/mathlib/math
Battery|lib/battery
PWM_Output|drivers/pwm_out
RC_Update|lib/rc modules/rc_update
Navigator|modules/navigator
EKF2|modules/ekf2/EKF
Magnetometer|modules/sensors/vehicle_magnetometer
Barometer|modules/sensors/vehicle_air_data
VTOL|modules/vtol_att_control'
echo "== (1) codigo real do PX4"
echo "$MODS" | while IFS='|' read -r nome dirs; do
  cd "$PX4/src"; alvos=""; for d in $dirs; do alvos="$alvos $d"; done
  lg="$REPO/$OUT/real_$nome.txt"
  { echo "# cppcheck $FLAGS -i '*[Tt]est*' $alvos"; } > "$lg"
  t0=$(date +%s); timeout 900 cppcheck $FLAGS -i'*[Tt]est*' $alvos >> "$lg" 2>&1; echo "# exit: $? tempo: $(( $(date +%s)-t0 ))s" >> "$lg"
  n=$(grep -c ':\(error\|warning\|performance\|portability\):' "$lg"); e=$(grep -c ':error:' "$lg")
  printf '%-13s achados=%-4s (erro=%s) | %s\n' "$nome" "$n" "$e" "$(tail -1 "$lg")" | tee -a "$REPO/$OUT/RESUMO_real.txt"
done
cd "$REPO"
echo "== (2) harnesses que o ESBMC verificou"
for h in GPS/test_gps_driver IMU_BMI088/test_bmi088_imu MAVLink/test_mavlink_parser Expo_RC/test_expo_mathlib Battery_Monitor/test_battery PWM_Output/test_pwm_out RC_Update/test_rc_update Navigator_Mission/test_navigator_mission EKF2_Covariance/test_ekf2_covariance Magnetometer/test_vehicle_magnetometer Barometer/test_barometer VTOL/test_vtol_type; do
  nome=$(echo "$h" | cut -d/ -f1); lg="$OUT/harness_$nome.txt"
  { echo "# cppcheck $FLAGS resultados/$h.cpp"; } > "$lg"
  timeout 300 cppcheck $FLAGS "resultados/$h.cpp" >> "$lg" 2>&1; echo "# exit: $?" >> "$lg"
  n=$(grep -c ':\(error\|warning\|performance\|portability\):' "$lg"); e=$(grep -c ':error:' "$lg")
  printf '%-18s achados=%-3s (erro=%s)\n' "$nome" "$n" "$e" | tee -a "$OUT/RESUMO_harness.txt"
done
