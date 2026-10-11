#!/usr/bin/env bash
# Roda os 12 harnesses da Tabela II com um binario do ESBMC e compara com os
# logs de julho (medianas_logs/, ESBMC 8.1.0 + Boolector 3.2.4).
#
# Uso (na raiz do repositorio):
#   comparacao_versoes/comparar_versoes.sh <rotulo> <binario_esbmc> [timeout_shell_s] [timeout_esbmc]
#   ex.: comparacao_versoes/comparar_versoes.sh v8.5_nuvem /opt/tools/esbmc-8.5/bin/esbmc
#
# Flags: as do artigo (comando padrao + ajustes de MAVLink, Navigator e VTOL),
# sempre com Boolector. Saida: comparacao_versoes/logs/<rotulo>/<modulo>.log e
# uma tabela com veredito, VCCs e contraexemplo lado a lado com julho.
# So o VEREDITO, as VCCs e o contraexemplo sao comparaveis entre maquinas;
# os tempos nao.
set -u
ROT="${1:?rotulo}"; E="${2:?binario do esbmc}"; TO="${3:-1800}"
# Timeout interno do ESBMC: 120s no artigo (iartes). Numa maquina mais lenta o
# GPS (65 s no iartes) pode estourar; passe p.ex. 600s como 4o argumento.
# Timeout = SEM VEREDITO, nunca "passou".
ETO="${4:-120s}"
cd "$(dirname "$0")/.."
OUT="comparacao_versoes/logs/$ROT"; mkdir -p "$OUT"
BASE=(--no-unwinding-assertions --overflow-check --default-solver boolector --timeout "$ETO")

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
ce() { grep -aoE '(count|input_len|temp_msb|temp_lsb) = [0-9]+|\.len=[1-9][0-9]*' "$1" | tr '\n' ' '; }

printf "%-13s | %-34s | %-34s | %s\n" modulo "julho 8.1.0 (veredito/VCC)" "$ROT (veredito/VCC)" "contraexemplo igual?"
for m in $(printf '%s\n' "${!ARQ[@]}" | sort); do
  lg="$OUT/$m.log"
  { echo "# data: $(date -u +%FT%TZ)  host: $(hostname)  cpu:$(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2-)"
    echo "# versao: $("$E" --version 2>&1 | head -1)"
    echo "# comando: esbmc ${ARQ[$m]} $(extra $m) ${BASE[*]}"; } > "$lg"
  timeout "$TO" "$E" "${ARQ[$m]}" $(extra "$m") "${BASE[@]}" >> "$lg" 2>&1
  echo "# exit: $?" >> "$lg"
  j="medianas_logs/${m}_run1.log"
  vj="$(grep -aoE 'VERIFICATION [A-Z]+' "$j" | tail -1 | awk '{print $2}')/$(grep -aoE 'Generated [0-9]+' "$j" | awk '{print $2}')"
  vn="$(grep -aoE 'VERIFICATION [A-Z]+' "$lg" | tail -1 | awk '{print $2}')/$(grep -aoE 'Generated [0-9]+' "$lg" | awk '{print $2}')"
  [ "$vn" = "/" ] && vn="SEM VEREDITO ($(grep -aoE 'ERROR: .*' "$lg" | head -1))"
  cj="$(ce "$j")"; cn="$(ce "$lg")"
  igual=$([ "$cj" = "$cn" ] && echo "sim" || echo "NAO: julho[$cj] x [$cn]")
  printf "%-13s | %-34s | %-34s | %s\n" "$m" "$vj" "$vn" "$igual"
done
