#!/usr/bin/env bash
# Executa o ESBMC sobre um harness + arquivos ORIGINAIS do PX4, 3 vezes,
# e registra no log: data, máquina, versão do ESBMC, commit do PX4 e a
# linha de comando completa (lição do artigo anterior).
#
# Uso:
#   PX4_ROOT=../px4 ./run_esbmc.sh <rótulo> <harness.cpp> "<fontes PX4>" "<-I extras>" [opções ESBMC...]
# Exemplo (PCA9685, teste 1, com asserções de desdobramento):
#   PX4_ROOT=../px4 ./run_esbmc.sh pca9685_T1_ua ../harnesses/pca9685/test_pca9685.cpp \
#     "src/drivers/pca9685_pwm_out/PCA9685.cpp" "-I src/drivers/pca9685_pwm_out" \
#     -DTEST_CHOICE=1 --unwind 20 --unwinding-assertions --overflow-check \
#     --default-solver boolector --timeout 300s
set -uo pipefail
RUNS=${RUNS:-3}
ESBMC=${ESBMC:-esbmc}   # caminho do ESBMC (ex.: ESBMC=~/esbmc/bin/esbmc)
if ! command -v "$ESBMC" >/dev/null 2>&1; then
  echo "ERRO: ESBMC não encontrado ($ESBMC). Instale ou defina ESBMC=/caminho/para/esbmc" >&2; exit 2; fi
PX4_ROOT=${PX4_ROOT:?defina PX4_ROOT}
label=$1; harness=$2; srcs=$3; incs=$4; shift 4
here="$(cd "$(dirname "$0")" && pwd)"
# pasta da rodada: LOGDIR (se definido) ou logs/<data>_<maquina>_esbmc<versão>
ver=$("$ESBMC" --version 2>&1 | grep -oE "[0-9]+\.[0-9]+(\.[0-9]+)?" | head -1)
logdir=${LOGDIR:-"$here/../logs/$(date +%Y-%m-%d)_$(hostname -s)_esbmc${ver}"}
mkdir -p "$logdir"

src_args=(); for f in $srcs; do src_args+=("$PX4_ROOT/$f"); done
[ ${#src_args[@]} -eq 0 ] && src_args=()
inc_args=(); set -f; for t in $incs; do
  if [ "$t" = "-I" ]; then continue; fi; inc_args+=(-I "$PX4_ROOT/$t"); done; set +f
inc_args+=(-I "$here/../stubs")

cmd=("$ESBMC" "$harness" "${src_args[@]}" "${inc_args[@]}" "$@")
times=()
for i in $(seq 1 "$RUNS"); do
  log="$logdir/${label}_run${i}.log"
  {
    echo "### data: $(date -Iseconds)"
    echo "### maquina: $(uname -srm); $(grep -m1 'model name' /proc/cpuinfo 2>/dev/null | cut -d: -f2)"
    echo "### esbmc: $("$ESBMC" --version 2>&1 | head -1)"
    echo "### px4 commit: $(git -C "$PX4_ROOT" rev-parse HEAD 2>/dev/null || cat "$PX4_ROOT/.px4_commit" 2>/dev/null || echo desconhecido)"
    echo "### comando: ${cmd[*]}"
    echo
  } > "$log"
  "${cmd[@]}" >> "$log" 2>&1
  t=$(grep -aoE "BMC program time: [0-9.]+" "$log" | grep -aoE "[0-9.]+" | tail -1)
  [ -z "$t" ] && t=$(grep -aoE "Runtime decision procedure: [0-9.]+" "$log" | grep -aoE "[0-9.]+" | tail -1)
  v=$(grep -aoE "VERIFICATION (SUCCESSFUL|FAILED)|Timed out|PARSING ERROR|CONVERSION ERROR" "$log" | tail -1)
  printf '  run %d: %ss  [%s]\n' "$i" "${t:-NA}" "${v:-sem veredito}"
  times+=("${t:-NA}")
done
med=$(printf '%s\n' "${times[@]}" | grep -v NA | sort -n | awk '{v[NR]=$1} END{if(NR)printf "%.3f", v[int((NR+1)/2)]; else printf "NA"}')
echo "  >> $label: mediana = ${med}s"
