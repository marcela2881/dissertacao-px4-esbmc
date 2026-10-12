#!/usr/bin/env bash
# Roda os 12 harnesses da Tabela II com as MESMAS flags do artigo, mas SEM
# --no-unwinding-assertions (assertions de desenrolamento ligadas).
# ESBMC 8.1.0 + Boolector. Uso: verificacao_unwinding_2026-10/rodar_com_unwinding.sh [binario] [timeout_esbmc]
set -u
E="${1:-/opt/tools/esbmc-8.1/bin/esbmc}"; ETO="${2:-600s}"
cd "$(dirname "$0")/.."
OUT=verificacao_unwinding_2026-10/campanha_completa; mkdir -p "$OUT"
declare -A ARQ=(
  [Barometer]=resultados/Barometer/test_barometer.cpp
  [Battery]=resultados/Battery_Monitor/test_battery.cpp
  [EKF2]=resultados/EKF2_Covariance/test_ekf2_covariance.cpp
  [Expo]=resultados/Expo_RC/test_expo_mathlib.cpp
  [GPS]=resultados/GPS/test_gps_driver.cpp
  [IMU_BMI088]=resultados/IMU_BMI088/test_bmi088_imu.cpp
  [MAVLink]=resultados/MAVLink/test_mavlink_parser.cpp
  [Magnetometer]=resultados/Magnetometer/test_vehicle_magnetometer.cpp
  [Navigator]=resultados/Navigator_Mission/test_navigator_mission.cpp
  [PWM]=resultados/PWM_Output/test_pwm_out.cpp
  [RC]=resultados/RC_Update/test_rc_update.cpp
  [VTOL]=resultados/VTOL/test_vtol_type.cpp
)
extra() { case "$1" in MAVLink) echo "--unwind 300";; Navigator) echo "--unwind 15";;
                       VTOL) echo "--unwind 10 --no-library";; *) echo "--unwind 10";; esac; }
for m in $(printf '%s\n' "${!ARQ[@]}" | sort); do
  lg="$OUT/$m.log"
  { echo "# data: $(date -u +%FT%TZ)  host: $(hostname)"
    echo "# versao: $("$E" --version 2>&1 | head -1)"
    echo "# comando: esbmc ${ARQ[$m]} $(extra $m) --overflow-check --default-solver boolector --timeout $ETO   (SEM --no-unwinding-assertions)"; } > "$lg"
  timeout 1800 "$E" "${ARQ[$m]}" $(extra "$m") --overflow-check --default-solver boolector --timeout "$ETO" >> "$lg" 2>&1
  echo "# exit: $?" >> "$lg"
  v=$(grep -aoE 'VERIFICATION [A-Z]+' "$lg" | tail -1); p=$(grep -a -A2 'Violated property' "$lg" | sed -n 3p | sed 's/^ *//')
  printf "%-13s | %-25s | %s\n" "$m" "${v:-SEM VEREDITO}" "$p"
done | tee "$OUT/RESUMO.txt"
