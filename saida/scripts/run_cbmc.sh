#!/usr/bin/env bash
# Executa o CBMC N vezes (RUNS, padrão 3) e registra no log: data, máquina,
# versão do CBMC, commit do PX4 e a linha de comando. Tempo = relógio de parede
# do processo (o CBMC não tem um "tempo total" único como o ESBMC).
# Uso: PX4_ROOT=.. ./run_cbmc.sh <rótulo> "<arquivos .c/.cpp>" "<-I extras>" [opções CBMC...]
set -uo pipefail
RUNS=${RUNS:-3}
CBMC=${CBMC:-cbmc}
TIMEOUT_S=${TIMEOUT_S:-600}
command -v "$CBMC" >/dev/null 2>&1 || { echo "ERRO: CBMC não encontrado (sudo apt install -y cbmc)" >&2; exit 2; }
label=$1; files=$2; incs=$3; shift 3
here="$(cd "$(dirname "$0")" && pwd)"
ver=$("$CBMC" --version 2>&1 | grep -oE "[0-9]+\.[0-9]+(\.[0-9]+)?" | head -1)
logdir=${LOGDIR:?defina LOGDIR}; mkdir -p "$logdir"
inc_args=(); for t in $incs; do inc_args+=(-I "$t"); done
inc_args+=(-I "$here/../stubs")
cmd=(timeout "$TIMEOUT_S" "$CBMC" $files "${inc_args[@]}" "$@")
times=()
for i in $(seq 1 "$RUNS"); do
  log="$logdir/${label}_run${i}.log"
  {
    echo "### data: $(date -Iseconds)"
    echo "### maquina: $(uname -srm); $(grep -m1 'model name' /proc/cpuinfo 2>/dev/null | cut -d: -f2)"
    echo "### cbmc: $("$CBMC" --version 2>&1 | head -1)"
    echo "### px4 commit: $(cat "${PX4_ROOT:-.}/.px4_commit" 2>/dev/null || echo desconhecido)"
    echo "### comando: ${cmd[*]}"
    echo
  } > "$log"
  t0=$EPOCHREALTIME
  "${cmd[@]}" >> "$log" 2>&1
  rc=$?
  t1=$EPOCHREALTIME
  echo "### codigo de saida do cbmc: $rc" >> "$log"
  t=$(awk -v a="$t0" -v b="$t1" 'BEGIN{printf "%.3f", b-a}')
  v=$(grep -aoE "VERIFICATION (SUCCESSFUL|FAILED)|PARSING ERROR|CONVERSION ERROR" "$log" | tail -1)
  [ "$rc" -eq 124 ] && v="Timed out"
  [ -z "$v" ] && v=$(grep -aoE "(error|ERROR): .{0,60}" "$log" | head -1)
  [ -z "$v" ] && v="sem veredito (codigo $rc)"
  case "$v" in VERIFICATION*) ;; *) t=NA ;; esac
  printf '  run %d: %ss  [%s]\n' "$i" "$t" "$v"
  times+=("$t")
done
med=$(printf '%s\n' "${times[@]}" | grep -v NA | sort -n | awk '{v[NR]=$1} END{if(NR)printf "%.3f", v[int((NR+1)/2)]; else printf "NA"}')
echo "  >> $label: mediana = ${med}s"
