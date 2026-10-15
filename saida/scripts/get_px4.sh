#!/usr/bin/env bash
# Baixa o PX4-Autopilot no commit fixado do estudo (sem modificar nada).
# Uso: ./get_px4.sh [destino]   (padrão: ../px4)
set -euo pipefail
COMMIT=8be2590c1529517ccd54fbc09ae138402517baf4
DEST=${1:-"$(dirname "$0")/../px4"}
if [ ! -d "$DEST/.git" ]; then
  git clone --filter=blob:none --no-checkout https://github.com/PX4/PX4-Autopilot.git "$DEST"
fi
git -C "$DEST" checkout -q "$COMMIT"
echo "PX4 em $DEST, commit $(git -C "$DEST" rev-parse HEAD)"
