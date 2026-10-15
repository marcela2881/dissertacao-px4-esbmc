#!/usr/bin/env bash
# Roda os 5 testes do PCA9685, com e sem asserções de desdobramento,
# 3 execuções cada (mediana), usando Boolector. Logs em ../logs/.
# Uso: ./run_pca9685_all.sh            (PX4 em ../px4; baixa se não existir)
set -uo pipefail
cd "$(dirname "$0")"
ESBMC=${ESBMC:-esbmc}; export ESBMC
command -v "$ESBMC" >/dev/null 2>&1 || { echo "ERRO: ESBMC não encontrado. Defina ESBMC=/caminho/para/esbmc"; exit 2; }
"$ESBMC" --version
[ -d ../px4/src/drivers ] || { rm -rf ../px4; ./get_px4.sh ../px4; }
export PX4_ROOT=../px4
H=../harnesses/pca9685/test_pca9685.cpp
SRC=""   # o harness inclui o PCA9685.cpp original (unidade única)
INC="src/drivers/pca9685_pwm_out"
COMMON=(--unwind 70 --overflow-check --default-solver boolector --timeout 600s)
for t in 0 1 2 3 4; do
  echo "=== T$t com asserções de desdobramento (padrão)"
  ./run_esbmc.sh pca9685_T${t}_ua  "$H" "$SRC" "$INC" -DTEST_CHOICE=$t "${COMMON[@]}"
  echo "=== T$t sem asserções de desdobramento"
  ./run_esbmc.sh pca9685_T${t}_nua "$H" "$SRC" "$INC" -DTEST_CHOICE=$t "${COMMON[@]}" --no-unwinding-assertions
done
