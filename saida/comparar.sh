#!/usr/bin/env bash
# Compara ESBMC × CBMC no MESMO harness, com as MESMAS pré-condições, o MESMO
# limite de desdobramento e o MESMO tempo limite, na mesma máquina.
#
#   ./comparar.sh leva1    rapido|oficial   -> 12 módulos do artigo (pasta leva1/)
#   ./comparar.sh <módulo> rapido|oficial   -> um módulo de modulos/ (ex.: voxl_esc)
#   ./comparar.sh todos    rapido|oficial   -> leva1 + todos os módulos de modulos/
#
# rapido : 1 execução, só com asserções de desdobramento (UA), limite 300 s
# oficial: 3 execuções (mediana), com (UA) e sem (NUA), limite 600 s
#
# Equivalência de opções (ESBMC 8.5 -> CBMC 5.95):
#   checagens padrão do ESBMC (limites, ponteiros, divisão por zero)
#                          -> --bounds-check --pointer-check --div-by-zero-check
#   --overflow-check       -> --signed-overflow-check --float-overflow-check
#   --ub-shift-check       -> --undefined-shift-check
#   --nan-check, --unwind, --function, --no-library, -D...  -> iguais
#   UA: ESBMC padrão / CBMC --unwinding-assertions ; NUA: ESBMC --no-unwinding-assertions / CBMC padrão
#   Harness C++ no CBMC: --cpp11. 1ª leva: shim cbmc/shim_esbmc_assume.cpp (__ESBMC_assume -> __CPROVER_assume).
# Tempo: ESBMC = "BMC program time"; CBMC = relógio de parede do processo.
# CBMC + C++: a glibc do Ubuntu 24.04 usa _Float32/_Float64/... no <math.h> quando o compilador é
# recente; o frontend C++ do CBMC 5.95 não conhece esses tipos ("symbol '_Float32' is unknown",
# CONVERSION ERROR, PC do Bruno 18/10/2026). Contorno só no CBMC, sem tocar nos harnesses:
# -D_Float32=float -D_Float64=double -D_Float32x=double -D_Float64x=double -D_Float128=double
# (os tipos estendidos só aparecem em declarações da glibc não usadas pelos harnesses).
set -uo pipefail
cd "$(dirname "$0")"
raiz="$(pwd)"
alvo=${1:-}; modo=${2:-rapido}
[ -n "$alvo" ] || { echo "Uso: ./comparar.sh leva1|<módulo>|todos rapido|oficial"; exit 1; }

if [ -z "${ESBMC:-}" ]; then ESBMC=$(find "$HOME/esbmc-8.5" -name esbmc -type f 2>/dev/null | head -1); [ -n "$ESBMC" ] || ESBMC=$(command -v esbmc || true); fi
[ -n "$ESBMC" ] && [ -x "$ESBMC" ] || { echo "ERRO: ESBMC não encontrado"; exit 2; }
CBMC=${CBMC:-$(command -v cbmc || true)}
[ -n "$CBMC" ] || { echo "ERRO: CBMC não encontrado. Instale com: sudo apt install -y cbmc"; exit 2; }
export ESBMC CBMC
[ -L px4 ] && [ ! -e px4 ] && rm -f px4
[ -d px4/src ] && [ -f px4/.px4_commit ] || ./scripts/get_px4.sh "$raiz/px4" || exit 3
export PX4_ROOT="$raiz/px4"

FLOATN="-D_Float32=float -D_Float64=double -D_Float32x=double -D_Float64x=double -D_Float128=double"

case "$modo" in
  rapido)  export RUNS=1; TO=300; VARIANTES="ua" ;;
  oficial) export RUNS=3; TO=600; VARIANTES="ua nua" ;;
  *) echo "modo: rapido ou oficial"; exit 1 ;;
esac
export TIMEOUT_S=$TO
vE=$("$ESBMC" --version 2>&1 | grep -oE "[0-9]+\.[0-9]+(\.[0-9]+)?" | head -1)
vC=$("$CBMC" --version 2>&1 | grep -oE "[0-9]+\.[0-9]+(\.[0-9]+)?" | head -1)
base="$raiz/logs/$(date +%Y-%m-%d)_$(hostname -s)_comparacao_${modo}"
mkdir -p "$base/esbmc" "$base/cbmc"
resumo="$base/RESUMO_COMPARACAO_${alvo}.txt"
{
  echo "===== COMPARAÇÃO ESBMC × CBMC: $alvo ($modo) ====="
  echo "maquina: $(hostname -s) | ESBMC $vE (boolector) | CBMC $vC (minisat) | timeout ${TO}s | $(date '+%d/%m/%Y %H:%M')"
  printf '%-34s | %-28s | %-28s\n' "teste" "ESBMC (veredito, mediana)" "CBMC (veredito, mediana)"
} | tee "$resumo"

# roda um par ESBMC/CBMC; $1=rótulo $2=harness $3=arquivos extras só do CBMC
# $4=pastas do PX4 (relativas ao PX4, iguais nos dois) $5=opções ESBMC $6=opções CBMC
par() {
  local label=$1 h=$2 extra=$3 inc=$4 oe=$5 oc=$6 le lc incabs="" i
  for i in $inc; do incabs="$incabs $PX4_ROOT/$i"; done
  echo "--- $label"
  le=$(LOGDIR="$base/esbmc" ./scripts/run_esbmc.sh "$label" "$h" "" "$inc" $oe --timeout ${TO}s 2>&1 | tee /dev/stderr | tail -"$((RUNS+1))")
  lc=$(LOGDIR="$base/cbmc" ./scripts/run_cbmc.sh "$label" "$h $extra" "$incabs" $oc 2>&1 | tee /dev/stderr | tail -"$((RUNS+1))")
  resum() { local med v; med=$(echo "$1" | grep -oE "mediana = [0-9.NA]+s" | grep -oE "[0-9.]+s|NAs"); v=$(echo "$1" | grep -oE "\[[^]]*\]" | sort | uniq -c | sort -rn | head -1 | sed 's/^ *//'); echo "$v, $med"; }
  printf '%-34s | %-28s | %-28s\n' "$label" "$(resum "$le")" "$(resum "$lc")" >> "$resumo"
}

leva1() {
  while IFS='|' read -r nome arq unw extra; do
    [[ -z "$nome" || "$nome" == \#* ]] && continue
    for v in $VARIANTES; do
      oe="--unwind $unw $extra --overflow-check --default-solver boolector"
      oc="--cpp11 $FLOATN --unwind $unw $extra --bounds-check --pointer-check --div-by-zero-check --signed-overflow-check --float-overflow-check"
      [ "$v" = nua ] && oe="$oe --no-unwinding-assertions" || oc="$oc --unwinding-assertions"
      par "leva1_${nome}_${v}" "$raiz/leva1/$arq" "$raiz/cbmc/shim_esbmc_assume.cpp" "" "$oe" "$oc"
    done
  done < leva1/leva1.conf
}

modulo() {
  local pasta; pasta=$(ls -d modulos/*/ | sed 's#/$##' | grep -E "modulos/(${1}_|[0-9]+_${1}$|${1}$)" | head -1)
  [ -f "$pasta/config.sh" ] || { echo "módulo $1 não encontrado"; return; }
  NOME=""; HARNESS=""; INC=""; TESTES=""; OPCOES=""; for n in 0 1 2 3 4 5 6 7 8 9; do unset "EXTRA_$n"; done
  source "$pasta/config.sh"
  [ -n "${SO_TESTES:-}" ] && TESTES="$SO_TESTES"
  local cpp=""; [[ "$HARNESS" == *.cpp ]] && cpp="--cpp11 $FLOATN"
  for t in $TESTES; do
    local ev="EXTRA_$t"; local ex=${!ev:-}
    local oc="" tok
    for tok in $OPCOES $ex; do
      case "$tok" in
        --overflow-check) oc="$oc --signed-overflow-check --float-overflow-check" ;;
        --ub-shift-check) oc="$oc --undefined-shift-check" ;;
        *) oc="$oc $tok" ;;
      esac
    done
    for v in $VARIANTES; do
      local oe="-DTEST_CHOICE=$t $OPCOES $ex --default-solver boolector" occ="$cpp -D TEST_CHOICE=$t $oc --bounds-check --pointer-check --div-by-zero-check"
      [ "$v" = nua ] && oe="$oe --no-unwinding-assertions" || occ="$occ --unwinding-assertions"
      par "${NOME}_T${t}_${v}" "$raiz/$pasta/$HARNESS" "" "$INC" "$oe" "$occ"
    done
  done
}

case "$alvo" in
  leva1) leva1 ;;
  todos) leva1; for p in modulos/*/; do [ -f "$p/config.sh" ] && modulo "$(basename "$p")"; done ;;
  *) modulo "$alvo" ;;
esac
echo; cat "$resumo"; echo "(logs: $base)"
