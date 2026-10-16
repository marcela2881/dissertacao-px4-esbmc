#!/usr/bin/env bash
# Roda as verificações de UM módulo.
#
#   ./rodar_modulo.sh <módulo> rapido    1 execução por teste, só com asserções de
#                                        desdobramento, timeout 300 s (achar problema de ferramenta)
#   ./rodar_modulo.sh <módulo> oficial   3 execuções (mediana), com E sem asserções de
#                                        desdobramento, timeout 600 s (resultado da pesquisa)
#
# <módulo> pode ser o número (02), o nome (roboclaw) ou a pasta (02_roboclaw).
# No fim aparece um RESUMO para colar na conversa.
set -uo pipefail
cd "$(dirname "$0")"
raiz="$(pwd)"

alvo=${1:-}; modo=${2:-rapido}
[ -n "$alvo" ] || { echo "Uso: ./rodar_modulo.sh <módulo> [rapido|oficial]"; ls modulos; exit 1; }
pasta=$(ls -d modulos/*/ | sed 's#/$##' | grep -E "modulos/(${alvo}_|[0-9]+_${alvo}$|${alvo}$)" | head -1)
[ -n "$pasta" ] && [ -f "$pasta/config.sh" ] || { echo "ERRO: módulo '$alvo' não encontrado ou sem config.sh. Disponíveis:"; ls modulos; exit 1; }
NOME=""; HARNESS=""; INC=""; TESTES=""; OPCOES=""
# shellcheck disable=SC1090
source "$pasta/config.sh"

# ESBMC: variável ESBMC, ou ~/esbmc-8.5, ou o que estiver no PATH
if [ -z "${ESBMC:-}" ]; then
  ESBMC=$(find "$HOME/esbmc-8.5" -name esbmc -type f 2>/dev/null | head -1)
  [ -n "$ESBMC" ] || ESBMC=$(command -v esbmc || true)
fi
[ -n "$ESBMC" ] && [ -x "$ESBMC" ] || { echo "ERRO: ESBMC não encontrado. Rode: export ESBMC=/caminho/para/esbmc"; exit 2; }
export ESBMC
SOLVER=${SOLVER:-boolector}

# PX4 no commit fixado
[ -L px4 ] && [ ! -e px4 ] && rm -f px4   # link quebrado vindo de outra máquina
[ -d px4/src ] || ./scripts/get_px4.sh "$raiz/px4" || { echo "ERRO ao baixar o PX4"; exit 3; }
export PX4_ROOT="$raiz/px4"

ver=$("$ESBMC" --version 2>&1 | grep -oE "[0-9]+\.[0-9]+(\.[0-9]+)?" | head -1)
export LOGDIR="$raiz/logs/$(date +%Y-%m-%d)_$(hostname -s)_esbmc${ver}_${modo}"
mkdir -p "$LOGDIR"
resumo="$LOGDIR/RESUMO_${NOME}.txt"

case "$modo" in
  rapido)  export RUNS=1; TIMEOUT=300s; VARIANTES="ua" ;;
  oficial) export RUNS=3; TIMEOUT=600s; VARIANTES="ua nua" ;;
  *) echo "Modo deve ser 'rapido' ou 'oficial'"; exit 1 ;;
esac

{
  echo "===== RESUMO: $NOME ($modo) ====="
  echo "maquina: $(hostname -s) | ESBMC $ver | solver $SOLVER | $(date '+%d/%m/%Y %H:%M')"
} | tee "$resumo"

for t in $TESTES; do
  extra_var="EXTRA_$t"; extra=${!extra_var:-}
  for v in $VARIANTES; do
    opts=(-DTEST_CHOICE="$t" $OPCOES $extra --default-solver "$SOLVER" --timeout "$TIMEOUT")
    [ "$v" = nua ] && opts+=(--no-unwinding-assertions)
    label="${NOME}_T${t}_${v}"
    echo "--- $label"
    ./scripts/run_esbmc.sh "$label" "$raiz/$pasta/$HARNESS" "" "$INC" "${opts[@]}" | tee -a "$LOGDIR/.saida_${NOME}"
    linha=$(grep "^  >> $label" "$LOGDIR/.saida_${NOME}" | tail -1 | sed 's/^  >> //')
    veredito=$(grep -h "^  run" "$LOGDIR/.saida_${NOME}" | tail -"$RUNS" | grep -oE "\[[^]]*\]" | sort | uniq -c | tr -s ' ' | tr '\n' ' ')
    echo "$linha | $veredito" >> "$resumo"
  done
done
rm -f "$LOGDIR/.saida_${NOME}"
echo
cat "$resumo"
echo "(logs completos em: $LOGDIR)"
