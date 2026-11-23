#!/usr/bin/env bash
# Roda no WSL (ESBMC instalado) os tres experimentos pedidos pelo orientador:
#   A) Battery Monitor com ate 16 celulas (limite antigo: 12)
#   B) cada teste de cada harness em execucao separada
#   C) GPS com k cobrindo os 210 bytes, com asserções de desenrolamento ligadas
# Uso (na raiz do repositorio):  bash verificacao_pendencias_2026-11/rodar_pendencias_wsl.sh
# Opcional: PULAR_LONGOS=1 pula a parte C (pode levar horas).
set -u
cd "$(dirname "$0")/.."
OUT=verificacao_pendencias_2026-11/logs; mkdir -p "$OUT"
TMP=$(mktemp -d)
TO_CURTO=600     # segundos, partes A e B
TO_LONGO=7200    # segundos, parte C

# ---- ambiente
{ echo "data: $(date -u +%FT%TZ)"; echo "host: $(hostname)"; uname -a
  lscpu | grep -E 'Model name|^CPU\(s\)'; free -h | sed -n 1,2p
  grep PRETTY_NAME /etc/os-release; esbmc --version | head -1; cbmc --version | head -1; } > "$OUT/ambiente.txt" 2>&1

# ---- solver: boolector se estiver embutido, senao z3
echo 'int main(){return 0;}' > "$TMP/p.c"
if esbmc "$TMP/p.c" --default-solver boolector >/dev/null 2>&1; then SOLVER=boolector; else SOLVER=z3; fi
echo "solver usado: $SOLVER" | tee -a "$OUT/ambiente.txt"

roda() {  # roda <log> <timeout_s> <args do esbmc...>
  local lg=$1 to=$2; shift 2
  { echo "# data: $(date -u +%FT%TZ)  host: $(hostname)"
    echo "# versao: $(esbmc --version | head -1)"
    echo "# comando: esbmc $* --default-solver $SOLVER   (limite externo ${to}s)"; } > "$lg"
  local t0=$(date +%s)
  timeout "$to" esbmc "$@" --default-solver "$SOLVER" >> "$lg" 2>&1
  echo "# exit: $?   tempo_de_parede: $(( $(date +%s) - t0 ))s" >> "$lg"
}
veredito() { local v p; v=$(grep -aoE 'VERIFICATION [A-Z]+' "$1" | tail -1)
  p=$(grep -a -A2 'Violated property' "$1" | sed -n 3p | sed 's/^ *//')
  printf '%s | %s | %s\n' "$2" "${v:-SEM VEREDITO (timeout?)}" "$p"; }

# ======================= A) Battery ate 16 celulas =======================
echo "== A) Battery Monitor, n_cells ate 16"
sed -E 's/(n_cells *<= *)12/\116/g' resultados/Battery_Monitor/test_battery.cpp > "$TMP/test_battery_16.cpp"
echo "linhas alteradas: $(grep -c 'n_cells <= 16' "$TMP/test_battery_16.cpp") (esperado: 2)"
cp "$TMP/test_battery_16.cpp" "$OUT/test_battery_16.cpp"
for i in 1 2 3; do
  roda "$OUT/A_battery16_run$i.log" $TO_CURTO "$OUT/test_battery_16.cpp" --unwind 10 --overflow-check --no-unwinding-assertions --timeout 120s
  veredito "$OUT/A_battery16_run$i.log" "battery16 run$i" | tee -a "$OUT/RESUMO.txt"
done

# ======================= B) cada teste em separado =======================
echo "== B) cada teste de cada harness em execucao separada"
# nome | arquivo | variavel | n_testes | flags extras
LISTA='Barometer|resultados/Barometer/test_barometer.cpp|choice|5|--unwind 10
Battery|resultados/Battery_Monitor/test_battery.cpp|choice|5|--unwind 10
EKF2|resultados/EKF2_Covariance/test_ekf2_covariance.cpp|choice|5|--unwind 10
Expo|resultados/Expo_RC/test_expo_mathlib.cpp|test_choice|5|--unwind 10
GPS|resultados/GPS/test_gps_driver.cpp|test_choice|5|--unwind 10
IMU_BMI088|resultados/IMU_BMI088/test_bmi088_imu.cpp|test_choice|6|--unwind 10
Magnetometer|resultados/Magnetometer/test_vehicle_magnetometer.cpp|choice|5|--unwind 10
Navigator|resultados/Navigator_Mission/test_navigator_mission.cpp|choice|5|--unwind 15
PWM|resultados/PWM_Output/test_pwm_out.cpp|choice|5|--unwind 10
RC|resultados/RC_Update/test_rc_update.cpp|choice|5|--unwind 10
VTOL|resultados/VTOL/test_vtol_type.cpp|choice|5|--unwind 10 --no-library'
echo "$LISTA" | while IFS='|' read -r nome arq var n extra; do
  for ((k=0; k<n; k++)); do
    f="$TMP/${nome}_t$k.cpp"
    sed -E "s/int +$var *= *nondet_int\(\);/int $var = $k;/" "$arq" > "$f"
    if cmp -s "$f" "$arq"; then echo "AVISO: nao consegui fixar $var em $arq"; fi
    lg="$OUT/B_${nome}_teste$k.log"
    roda "$lg" $TO_CURTO "$f" $extra --overflow-check --no-unwinding-assertions --timeout 120s
    veredito "$lg" "$nome teste $k" | tee -a "$OUT/RESUMO.txt"
  done
done
echo "(MAVLink nao tem selecao de teste: o main e uma unica sequencia; nao se aplica.)" | tee -a "$OUT/RESUMO.txt"

# ======================= C) GPS com k = 211 =======================
if [ "${PULAR_LONGOS:-0}" = "1" ]; then echo "== C) pulado (PULAR_LONGOS=1)"; else
  echo "== C) GPS com k = 211 (cobre os 210 bytes), asserções de desenrolamento LIGADAS; pode demorar"
  roda "$OUT/C_gps_k211_original.log" $TO_LONGO resultados/GPS/test_gps_driver.cpp --unwind 211 --overflow-check
  veredito "$OUT/C_gps_k211_original.log" "GPS original k=211" | tee -a "$OUT/RESUMO.txt"
  roda "$OUT/C_gps_k211_prop_limpa.log" $TO_LONGO resultados/GPS/test_gps_prop_limpa.cpp --unwind 211 --overflow-check
  veredito "$OUT/C_gps_k211_prop_limpa.log" "GPS prop. limpa k=211" | tee -a "$OUT/RESUMO.txt"
fi

echo; echo "Pronto. Empacote e envie:  tar czf resultados_pendencias.tgz verificacao_pendencias_2026-11/logs"
