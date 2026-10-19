#!/usr/bin/env bash
# FASE 2 (validacao) em ESBMC 8.5 e CBMC 5.95.1, com o protocolo da fase 3.
# Harnesses: fase2/harnesses (copias de cbmc/ e seeded_tests/, sem alteracao) +
# gps_guarda_semeado.cpp (patch do GPS com a MESMA pre-condicao antes/depois).
#   rapido : 1 execucao, SEM unwinding assertions (como na fase 2 original), 300 s
#   oficial: 3 execucoes (mediana), sem e com unwinding assertions, 600 s
# Uso: ./rodar_fase2.sh rapido|oficial [nome]   (nome = so uma linha do fase2.conf)
set -uo pipefail
cd "$(dirname "$0")"
raiz="$(pwd)"; modo=${1:-rapido}; so=${2:-}
if [ -z "${ESBMC:-}" ]; then ESBMC=$(find "$HOME/esbmc-8.5" -name esbmc -type f 2>/dev/null | head -1); [ -n "$ESBMC" ] || ESBMC=$(command -v esbmc || true); fi
[ -n "$ESBMC" ] && [ -x "$ESBMC" ] || { echo "ERRO: ESBMC não encontrado"; exit 2; }
CBMC=${CBMC:-$(command -v cbmc || true)}
[ -n "$CBMC" ] || { echo "ERRO: CBMC não encontrado (sudo apt install -y cbmc)"; exit 2; }
export ESBMC CBMC PX4_ROOT="$raiz/px4"
case "$modo" in
  rapido)  export RUNS=1; TO=300; VARS="nua" ;;
  oficial) export RUNS=3; TO=600; VARS="nua ua" ;;
  *) echo "modo: rapido ou oficial"; exit 1 ;;
esac
export TIMEOUT_S=$TO
H="$raiz/fase2/harnesses"
base="$raiz/logs/$(date +%Y-%m-%d)_$(hostname -s)_fase2_${modo}"; mkdir -p "$base/esbmc" "$base/cbmc"
resumo="$base/RESUMO_FASE2.txt"
{
  echo "===== FASE 2 (validação) em ESBMC × CBMC ($modo) ====="
  echo "maquina: $(hostname -s) | $("$ESBMC" --version 2>&1 | head -1) | $("$CBMC" --version 2>&1 | head -1) | timeout ${TO}s | $(date '+%d/%m/%Y %H:%M')"
  printf '%-30s | %-26s | %-26s | %-26s\n' "teste" "ESBMC Boolector" "ESBMC Z3" "CBMC"
} > "$resumo"
resum() { local med v; med=$(echo "$1" | grep -oE "mediana = [0-9.NA]+s" | grep -oE "[0-9.]+s|NAs"); v=$(echo "$1" | grep -oE "\[[^]]*\]" | sort | uniq -c | sort -rn | head -1 | sed 's/^ *//'); echo "$v, $med"; }
viol() { grep -a -A3 "Violated property" "$1" 2>/dev/null | sed -n '2,3p' | tr -s ' ' | tr '\n' ' ' | cut -c1-150; grep -a "FAILURE" "$1" 2>/dev/null | head -2 | cut -c1-150; }
while IFS='|' read -r nome arq unw z3 extra desc; do
  [[ -z "$nome" || "$nome" == \#* ]] && continue
  [ -n "$so" ] && [ "$so" != "$nome" ] && continue
  for v in $VARS; do
    lab="${nome}_${v}"; echo "--- $lab  ($desc)"
    # ESBMC: __CPROVER_assume dos harnesses da fase 2 vira __ESBMC_assume (mesmo significado)
    oe=(-D'__CPROVER_assume(c)=__ESBMC_assume(c)' $extra --unwind "$unw" --overflow-check --timeout ${TO}s)
    oc=($extra --unwind "$unw" --bounds-check --pointer-check --div-by-zero-check --signed-overflow-check --float-overflow-check)
    [ "$v" = nua ] && oe+=(--no-unwinding-assertions) || oc+=(--unwinding-assertions)
    eb=$(LOGDIR="$base/esbmc" ./scripts/run_esbmc.sh "${lab}_boolector" "$H/$arq" "" "" "${oe[@]}" --default-solver boolector 2>&1)
    echo "$eb"; viol "$base/esbmc/${lab}_boolector_run1.log"
    ez="—"
    if [ "$z3" = z3 ]; then
      ez=$(LOGDIR="$base/esbmc" ./scripts/run_esbmc.sh "${lab}_z3" "$H/$arq" "" "" "${oe[@]}" --default-solver z3 2>&1); echo "$ez"
    fi
    hc="$H/$arq"
    if [[ "$arq" == *.cpp ]]; then
      hc="$base/cbmc/entrada_${lab}.cpp"
      printf '#include "%s"\n#include "%s"\n' "$raiz/cbmc/prelude_cbmc.h" "$H/$arq" > "$hc"
      oc=(--cpp11 "${oc[@]}")
    fi
    ec=$(LOGDIR="$base/cbmc" ./scripts/run_cbmc.sh "$lab" "$hc" "" "${oc[@]}" 2>&1)
    echo "$ec"; viol "$base/cbmc/${lab}_run1.log"
    zz="—"; [ "$z3" = z3 ] && zz=$(resum "$ez")
    printf '%-30s | %-26s | %-26s | %-26s\n' "$lab" "$(resum "$eb")" "$zz" "$(resum "$ec")" >> "$resumo"
  done
done < fase2/fase2.conf
echo; cat "$resumo"; echo "(logs: $base)"
