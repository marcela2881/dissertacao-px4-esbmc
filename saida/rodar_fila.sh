#!/usr/bin/env bash
# Roda em fila, UMA etapa por vez, tudo o que falta das rodadas rápidas.
# Feito para rodar em segundo plano:
#
#   nohup ./rodar_fila.sh > fila.txt 2>&1 &
#   tail -f fila.txt                      # acompanhar (Ctrl+C só para de olhar)
#   grep "^#####" fila.txt                # ver em que etapa está
#
# Se outra verificação (comparar.sh, rodar_modulo.sh, rodar_fase2.sh) ainda estiver
# rodando, espera ela terminar antes de começar, para não misturar os tempos.
# Etapa que falha não para a fila: o erro fica registrado e a próxima começa.
set -u
cd "$(dirname "$0")"

etapa() {  # <nome> <comando...>
  local nome="$1"; shift
  echo
  echo "##### $nome | início $(date '+%d/%m %H:%M')"
  "$@"
  local rc=$?
  echo "##### $nome | fim $(date '+%d/%m %H:%M') | código de saída $rc"
}

while pgrep -f "comparar.sh|rodar_modulo.sh|rodar_fase2.sh|rodar_tudo.sh" >/dev/null; do
  echo "aguardando outra verificação terminar ($(date '+%H:%M'))"
  sleep 120
done

etapa "1 replays (fase 1 e fase 3)"        ./replay/rodar_replay.sh
etapa "2 dshot D3 (51 tamanhos) ESBMC rápido" env SO_TESTES="$(seq -s " " 100 150)" ./rodar_modulo.sh dshot rapido
for m in voxl_esc pca9685 roboclaw tap_esc dshot; do
  etapa "3 $m ESBMC x CBMC rápido"         ./comparar.sh "$m" rapido
done
etapa "4 pacote v5 (ESBMC, CBMC e replay)" bash -c 'cd v5 && ./rodar_tudo.sh'

echo
echo "##### FILA TERMINADA $(date '+%d/%m %H:%M')"
