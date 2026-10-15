#!/usr/bin/env bash
# Baixa o PX4-Autopilot no commit fixado do estudo (sem modificar nada).
# Funciona com git OU, se não houver git, com curl/wget (arquivo .tar.gz do commit).
# Uso: ./get_px4.sh [destino]   (padrão: ../px4)
set -euo pipefail
COMMIT=8be2590c1529517ccd54fbc09ae138402517baf4
DEST=${1:-"$(dirname "$0")/../px4"}
if command -v git >/dev/null 2>&1; then
  if [ ! -d "$DEST/.git" ]; then
    git clone --filter=blob:none --no-checkout https://github.com/PX4/PX4-Autopilot.git "$DEST"
  fi
  git -C "$DEST" checkout -q "$COMMIT"
else
  URL="https://github.com/PX4/PX4-Autopilot/archive/$COMMIT.tar.gz"
  mkdir -p "$DEST"
  if command -v curl >/dev/null 2>&1; then curl -L "$URL" -o /tmp/px4_$COMMIT.tar.gz
  else wget -O /tmp/px4_$COMMIT.tar.gz "$URL"; fi
  tar -xzf /tmp/px4_$COMMIT.tar.gz -C "$DEST" --strip-components=1
  rm -f /tmp/px4_$COMMIT.tar.gz
  echo "$COMMIT" > "$DEST/.px4_commit"     # registro do commit (sem git)
fi
echo "PX4 em $DEST, commit $COMMIT"
