#!/usr/bin/env bash
# Roda TODOS os módulos de modulos/, um depois do outro.
#   ./rodar_tudo.sh rapido     ou     ./rodar_tudo.sh oficial
# Dica: na fm07, rode dentro do "screen" para não perder a rodada se a conexão cair.
cd "$(dirname "$0")"
modo=${1:-rapido}
for p in modulos/*/; do
  [ -f "$p/config.sh" ] || continue
  ./rodar_modulo.sh "$(basename "$p")" "$modo"
done
