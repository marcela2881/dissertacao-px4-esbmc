#!/usr/bin/env bash
# Baixa o PX4-Autopilot no commit fixado do estudo (sem modificar nada).
# Usa o arquivo .tar.gz do commit (um único download, que é RETOMADO se a
# conexão cair; o clone com git falhou no meio em conexão instável).
# Uso: ./get_px4.sh [destino]   (padrão: ../px4)
set -euo pipefail
COMMIT=8be2590c1529517ccd54fbc09ae138402517baf4
DEST=${1:-"$(dirname "$0")/../px4"}
URL="https://github.com/PX4/PX4-Autopilot/archive/$COMMIT.tar.gz"
TGZ="${TMPDIR:-/tmp}/px4_$COMMIT.tar.gz"
ok=0
for tentativa in 1 2 3 4 5; do
  echo "Baixando o PX4 (tentativa $tentativa) ..."
  if command -v curl >/dev/null 2>&1; then
    curl -L --retry 3 -C - -o "$TGZ" "$URL" && ok=1
  else
    wget -c -O "$TGZ" "$URL" && ok=1
  fi
  [ "$ok" = 1 ] && gzip -t "$TGZ" 2>/dev/null && break
  ok=0; sleep 5
done
[ "$ok" = 1 ] || { echo "ERRO: download do PX4 falhou; rode de novo (o download continua de onde parou)"; exit 1; }
rm -rf "$DEST"; mkdir -p "$DEST"
tar -xzf "$TGZ" -C "$DEST" --strip-components=1
rm -f "$TGZ"
echo "$COMMIT" > "$DEST/.px4_commit"     # registro do commit
echo "PX4 em $DEST, commit $COMMIT"
