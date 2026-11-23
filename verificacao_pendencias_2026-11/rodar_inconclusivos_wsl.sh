#!/usr/bin/env bash
# Repete, com limite de tempo maior, os testes que ficaram sem veredito com 120 s:
#   EKF2 teste 1; GPS testes 0 a 3; VTOL testes 0 e 1; Battery com 16 celulas.
# Uso (na raiz do repositorio):   bash verificacao_pendencias_2026-11/rodar_inconclusivos_wsl.sh
# Limite por execucao (padrao 900 s):  LIMITE=1800 bash ...
set -u
cd "$(dirname "$0")/.."
LIMITE=${LIMITE:-900}
OUT=verificacao_pendencias_2026-11/logs_inconclusivos; mkdir -p "$OUT"
TMP=$(mktemp -d)
{ echo "data: $(date -u +%FT%TZ)"; echo "host: $(hostname)"; lscpu | grep -E 'Model name|^CPU\(s\)'; free -h | sed -n 1,2p
  esbmc --version | head -1; echo "limite por execucao: ${LIMITE}s"; } > "$OUT/ambiente.txt" 2>&1

cat > "$TMP/p.c" <<'EOC'
#include <assert.h>
int nondet_int(void);
int main(void){ int x = nondet_int(); assert(x != 5); return 0; }
EOC
if [ -z "${SOLVER:-}" ]; then
  if esbmc "$TMP/p.c" --default-solver boolector 2>&1 | grep -q 'has not been built'; then SOLVER=z3; else SOLVER=boolector; fi
fi
echo "solver usado: $SOLVER" | tee -a "$OUT/ambiente.txt"

roda() {  # roda <log> <args do esbmc...>
  local lg=$1; shift
  { echo "# data: $(date -u +%FT%TZ)  host: $(hostname)"; echo "# versao: $(esbmc --version | head -1)"
    echo "# comando: esbmc $* --default-solver $SOLVER --timeout ${LIMITE}s"; } > "$lg"
  local t0=$(date +%s)
  timeout $((LIMITE + 120)) esbmc "$@" --default-solver "$SOLVER" --timeout "${LIMITE}s" >> "$lg" 2>&1
  echo "# exit: $?   tempo_de_parede: $(( $(date +%s) - t0 ))s" >> "$lg"
}
veredito() { local v p t; v=$(grep -aoE 'VERIFICATION [A-Z]+' "$1" | tail -1)
  p=$(grep -a -A2 'Violated property' "$1" | sed -n 3p | sed 's/^ *//'); t=$(grep -a '^# exit' "$1" | sed 's/# //')
  printf '%s | %s | %s | %s\n' "$2" "${v:-SEM VEREDITO}" "$p" "$t"; }

# ---- Battery com 16 celulas (1 execucao)
sed -E 's/(n_cells *<= *)12/\116/g' resultados/Battery_Monitor/test_battery.cpp > "$OUT/test_battery_16.cpp"
roda "$OUT/battery16.log" "$OUT/test_battery_16.cpp" --unwind 10 --overflow-check --no-unwinding-assertions
veredito "$OUT/battery16.log" "battery16" | tee -a "$OUT/RESUMO.txt"

# ---- testes sem veredito: nome | arquivo | variavel | k | flags extras
LISTA='EKF2|resultados/EKF2_Covariance/test_ekf2_covariance.cpp|choice|1|--unwind 10
GPS|resultados/GPS/test_gps_driver.cpp|test_choice|3|--unwind 10
GPS|resultados/GPS/test_gps_driver.cpp|test_choice|0|--unwind 10
GPS|resultados/GPS/test_gps_driver.cpp|test_choice|1|--unwind 10
GPS|resultados/GPS/test_gps_driver.cpp|test_choice|2|--unwind 10
VTOL|resultados/VTOL/test_vtol_type.cpp|choice|0|--unwind 10 --no-library
VTOL|resultados/VTOL/test_vtol_type.cpp|choice|1|--unwind 10 --no-library'
echo "$LISTA" | while IFS='|' read -r nome arq var k extra; do
  f="$TMP/${nome}_t$k.cpp"
  sed -E "s/int +$var *= *nondet_int\(\);/int $var = $k;/" "$arq" > "$f"
  cmp -s "$f" "$arq" && echo "AVISO: nao consegui fixar $var em $arq"
  lg="$OUT/${nome}_teste$k.log"
  roda "$lg" "$f" $extra --overflow-check --no-unwinding-assertions
  veredito "$lg" "$nome teste $k" | tee -a "$OUT/RESUMO.txt"
done
echo; echo "Pronto. Empacote e envie:  tar czf inconclusivos.tgz verificacao_pendencias_2026-11/logs_inconclusivos"
