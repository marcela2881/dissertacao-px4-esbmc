#!/usr/bin/env bash
# Reproduz TODA a evidência do reporte da RoboClaw (R1 e R3), com o código ORIGINAL do PX4
# e com a correção proposta (6_pr/correcao_roboclaw.patch):
#   1) ESBMC no original: R1 (FAILED), R2 (SUCCESSFUL, faixa normal) e R3 (FAILED)
#   2) replay: g++ executa o Roboclaw.cpp original com os valores do contraexemplo
#   3) reproduções mínimas (sem PX4), para colar na issue
#   4) correção: ESBMC R1 (opções da oficial) e R3 (só a propriedade do R3, original x corrigido),
#      esperado SUCCESSFUL no corrigido; replay (esperado: nada reproduzido)
# Saída na tela e em ../logs/<data>_<máquina>_reporte_roboclaw/
# Uso (dentro da pasta que tem px4/, stubs/ e scripts/, ex. ~/verificacoes ou ~/reporte):
#   ./reporte_roboclaw/rodar_reporte.sh
set -uo pipefail
cd "$(dirname "$0")/.."
raiz="$(pwd)"; R="$raiz/reporte_roboclaw"
out="logs/$(date +%Y-%m-%d)_$(hostname -s)_reporte_roboclaw"; mkdir -p "$out" build_replay
command -v g++ >/dev/null || { echo "ERRO: g++ não encontrado (sudo apt install -y g++)"; exit 2; }
command -v patch >/dev/null || { echo "ERRO: patch não encontrado (sudo apt install -y patch)"; exit 2; }
if [ -z "${ESBMC:-}" ]; then ESBMC=$(find "$HOME/esbmc-8.5" -name esbmc -type f 2>/dev/null | head -1); [ -n "$ESBMC" ] || ESBMC=$(command -v esbmc || true); fi
[ -n "$ESBMC" ] && [ -x "$ESBMC" ] || { echo "ERRO: ESBMC não encontrado"; exit 2; }
[ -d px4/src/drivers/roboclaw ] || { echo "ERRO: PX4 não encontrado em ./px4"; exit 3; }
export ESBMC PX4_ROOT="$raiz/px4" RUNS=1

# Roboclaw.cpp/.hpp com a correção aplicada, numa pasta à parte (o PX4 original não é alterado)
corr="$raiz/build_replay/roboclaw_corrigido"; rm -rf "$corr"; mkdir -p "$corr"
cp px4/src/drivers/roboclaw/Roboclaw.cpp px4/src/drivers/roboclaw/Roboclaw.hpp "$corr/"
patch -s -p4 -d "$corr" < "$R/6_pr/correcao_roboclaw.patch" || { echo "ERRO: a correção não aplicou"; exit 4; }
cp "$R/6_pr/correcao_roboclaw.patch" "$corr/normalizacao.diff"   # registrado no log do ESBMC

esbmc_teste() {   # $1=rótulo $2=teste [$3=sem_overflow] ; PRE_INC opcional = versão corrigida
  local rot=$1 t=$2 chk=(--overflow-check --ub-shift-check)   # opções da oficial
  # sem_overflow: só as checagens padrão do ESBMC (limites, ponteiros, divisão) e as asserções do harness
  [ "${3:-}" = sem_overflow ] && chk=()
  LOGDIR="$raiz/$out/esbmc" ./scripts/run_esbmc.sh "$rot" "$R/1_verificacao_esbmc/test_roboclaw.cpp" "" \
    "src/drivers/roboclaw" -DTEST_CHOICE="$t" --function harness_main --unwind 20 ${chk[@]+"${chk[@]}"} \
    --default-solver boolector --timeout 300s | grep "run 1"
  # mostra a propriedade violada, se houver (para não confundir R3 com o falso positivo do R0)
  grep -a -A3 "Violated property" "$raiz/$out/esbmc/${rot}_run1.log" 2>/dev/null | sed -n '2,4p' | sed 's/^/      /'
}

echo "##### 1/4 ESBMC no código ORIGINAL (esperado: T1 FAILED, T2 SUCCESSFUL, T3 FAILED)"
for t in 1 2 3; do echo -n "roboclaw_T${t}_original:"; esbmc_teste "roboclaw_T${t}_original" "$t"; done 2>&1 | tee "$out/1_esbmc_original.txt"
grep -a -A60 "Counterexample" "$out"/esbmc/roboclaw_T1_original_run1.log 2>/dev/null \
  | grep -aE "outputs|g_drive_byte|Violated|assert|line [0-9]+ function" | head -20 > "$out/1_contraexemplo_R1.txt"

echo; echo "##### 2/4 Replay com o Roboclaw.cpp ORIGINAL"
{
  echo "### data: $(date -Iseconds) | maquina: $(uname -srm) | $(g++ --version | head -1)"
  echo "### px4 commit: $(cat px4/.px4_commit 2>/dev/null || echo desconhecido)"
  g++ -std=c++17 -O0 -I stubs -I px4/src/drivers/roboclaw "$R/2_replay/replay_roboclaw.cpp" -o build_replay/replay_original -lm \
    && ./build_replay/replay_original
} 2>&1 | tee "$out/2_replay_original.txt"

echo; echo "##### 3/4 Reproduções mínimas (sem PX4)"
for m in minimo_r1 minimo_r3; do
  echo "--- $m"
  g++ -std=c++17 "$R/3_minimos/$m.cpp" -o "build_replay/$m" -lm && "./build_replay/$m"
done 2>&1 | tee "$out/3_minimos.txt"

echo; echo "##### 4/4 Com a CORREÇÃO (6_pr/correcao_roboclaw.patch)"
echo "--- ESBMC R1, opções da oficial (esperado: T1 SUCCESSFUL)"
PRE_INC="$corr" esbmc_teste roboclaw_T1_corrigido 1 2>&1 | tee "$out/4_esbmc_corrigido.txt"
# R3: assert do select() no harness + checagens padrão, SEM --overflow-check/--ub-shift-check.
# Com elas, o corrigido para no falso positivo já conhecido do R0 ("arithmetic overflow on shl" em
# swapBytesInt32, readEncoder), visto em 19/10/2026 -- não é o R3. Original e corrigido com as MESMAS opções.
echo "--- ESBMC R3, só a propriedade do R3 (esperado: original FAILED, corrigido SUCCESSFUL)"
{ esbmc_teste roboclaw_T3_R3_original 3 sem_overflow
  PRE_INC="$corr" esbmc_teste roboclaw_T3_R3_corrigido 3 sem_overflow; } 2>&1 | tee -a "$out/4_esbmc_corrigido.txt"
echo "--- Replay (esperado: bytes até 127, select() 0 vezes, 0 situações reproduzidas)"
{
  g++ -std=c++17 -O0 -I "$corr" -I stubs -I px4/src/drivers/roboclaw "$R/2_replay/replay_roboclaw.cpp" -o build_replay/replay_corrigido -lm \
    && ./build_replay/replay_corrigido
} 2>&1 | tee "$out/4_replay_corrigido.txt"

echo; echo "(tudo salvo em: $raiz/$out)"
