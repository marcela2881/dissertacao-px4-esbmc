#!/usr/bin/env bash
# Roda os 5 testes do RoboClaw, com e sem asserções de desdobramento,
# 3 execuções cada (mediana), usando Boolector. Logs em ../logs/.
# Uso: ./run_roboclaw_all.sh            (PX4 em ../px4; baixa se não existir)
set -uo pipefail
cd "$(dirname "$0")"
ESBMC=${ESBMC:-esbmc}; export ESBMC
command -v "$ESBMC" >/dev/null 2>&1 || { echo "ERRO: ESBMC não encontrado. Defina ESBMC=/caminho/para/esbmc"; exit 2; }
"$ESBMC" --version
ver=$("$ESBMC" --version 2>&1 | grep -oE "[0-9]+\.[0-9]+(\.[0-9]+)?" | head -1)
export LOGDIR=${LOGDIR:-"$(pwd)/../logs/$(date +%Y-%m-%d)_$(hostname -s)_esbmc${ver}"}
echo "Logs em: $LOGDIR"
[ -d ../px4/src/drivers/roboclaw ] || { rm -rf ../px4; ./get_px4.sh ../px4; }
export PX4_ROOT=../px4
mkdir -p "$LOGDIR"
# contorno de duas limitações do ESBMC (ver ../ferramenta/); diff registrado em cada log
./normaliza_roboclaw.sh "$PX4_ROOT" "$LOGDIR/fonte_normalizada_roboclaw"
export PRE_INC="$(cd "$LOGDIR/fonte_normalizada_roboclaw" && pwd)"
H=../harnesses/roboclaw/test_roboclaw.cpp
SRC=""   # o harness inclui o Roboclaw.cpp (unidade única; normalizado, ver acima)
INC="src/drivers/roboclaw"
COMMON=(--function harness_main --unwind 20 --overflow-check --ub-shift-check --default-solver "${SOLVER:-boolector}" --timeout 600s)
for t in 0 1 2 3 4; do
  extra=(); [ "$t" = 4 ] && extra=(--nan-check)
  echo "=== R$t com asserções de desdobramento (padrão)"
  ./run_esbmc.sh roboclaw_R${t}_ua  "$H" "$SRC" "$INC" -DTEST_CHOICE=$t "${COMMON[@]}" "${extra[@]}"
  echo "=== R$t sem asserções de desdobramento"
  ./run_esbmc.sh roboclaw_R${t}_nua "$H" "$SRC" "$INC" -DTEST_CHOICE=$t "${COMMON[@]}" "${extra[@]}" --no-unwinding-assertions
done
