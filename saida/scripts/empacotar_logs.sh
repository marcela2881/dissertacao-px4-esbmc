#!/usr/bin/env bash
# Compacta a pasta de logs (todas as rodadas) num arquivo para levar ao PC e subir no git.
# Uso: ./empacotar_logs.sh   -> cria ~/logs_saida_<data>.tar.gz
set -euo pipefail
cd "$(dirname "$0")/.."
out=~/logs_saida_$(date +%Y-%m-%d_%H%M).tar.gz
tar -czf "$out" logs
echo "Arquivo criado: $out"; ls -lh "$out"
