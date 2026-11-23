#!/usr/bin/env bash
# Uso (na raiz do repositorio):  bash contagem_linhas_extraidas_2026-11/rodar_contagem.sh ~/PX4-Autopilot
# O PX4 precisa estar no commit 8be2590c15:  (cd ~/PX4-Autopilot && git checkout 8be2590c15)
set -u
PX4="${1:?informe o caminho do PX4-Autopilot}"
cd "$(dirname "$0")/.." || exit 1
echo "commit do PX4: $(git -C "$PX4" rev-parse --short=10 HEAD)"
mkdir -p contagem_linhas_extraidas_2026-11/logs
python3 contagem_linhas_extraidas_2026-11/contar_linhas_extraidas.py "$PX4" resultados \
  | tee contagem_linhas_extraidas_2026-11/logs/contagem_$(date +%F).txt
