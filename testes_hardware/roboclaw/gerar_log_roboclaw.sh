#!/usr/bin/env bash
# Gera o log oficial da verificacao formal do RoboClaw (Bug 1 e Bug 2) + hashes.
# Salve este arquivo na MESMA pasta dos dois harness e rode:
#   bash gerar_log_roboclaw.sh
# NAO use "set -e": VERIFICATION FAILED devolve codigo != 0 de proposito.

cd "$(dirname "$0")"
OUT="$HOME/evidencia_roboclaw"
mkdir -p "$OUT"

for f in harness_roboclaw_bug1.cpp harness_roboclaw_bug2.cpp; do
  if [ ! -f "$f" ]; then
    echo "ERRO: $f nao esta nesta pasta."
    echo "Coloque este script na mesma pasta dos dois harness e rode de novo."
    exit 1
  fi
done

{
  echo "# RoboClaw - verificacao formal (maquina da Marcela)"
  date
  echo -n "ESBMC: "; esbmc --version 2>&1 | head -1
  echo -n "CBMC:  "; cbmc  --version 2>&1 | head -1
  echo
  echo "################ BUG 1: byte de velocidade > 127 ################"
  echo "== ESBMC sem correcao (espera VERIFICATION FAILED) =="
  esbmc harness_roboclaw_bug1.cpp --overflow-check 2>&1 | tail -3
  echo
  echo "== ESBMC com correcao (espera VERIFICATION SUCCESSFUL) =="
  esbmc harness_roboclaw_bug1.cpp -DCORRECAO --overflow-check 2>&1 | tail -3
  echo
  echo "== CBMC sem correcao (espera VERIFICATION FAILED) =="
  cbmc  harness_roboclaw_bug1.cpp --signed-overflow-check 2>&1 | tail -3
  echo
  echo "== CBMC com correcao (espera VERIFICATION SUCCESSFUL) =="
  cbmc  harness_roboclaw_bug1.cpp -DCORRECAO --signed-overflow-check 2>&1 | tail -3
  echo
  echo "################ BUG 2: erro de write() nao detectado ################"
  echo "== ESBMC sem correcao (espera VERIFICATION FAILED) =="
  esbmc harness_roboclaw_bug2.cpp --overflow-check 2>&1 | tail -3
  echo
  echo "== ESBMC com correcao (espera VERIFICATION SUCCESSFUL) =="
  esbmc harness_roboclaw_bug2.cpp -DCORRECAO --overflow-check 2>&1 | tail -3
  echo
  echo "== CBMC sem correcao (espera VERIFICATION FAILED) =="
  cbmc  harness_roboclaw_bug2.cpp --signed-overflow-check 2>&1 | tail -3
  echo
  echo "== CBMC com correcao (espera VERIFICATION SUCCESSFUL) =="
  cbmc  harness_roboclaw_bug2.cpp -DCORRECAO --signed-overflow-check 2>&1 | tail -3
} | tee "$OUT/roboclaw_maquina.txt"

cp harness_roboclaw_bug1.cpp harness_roboclaw_bug2.cpp "$OUT/"
cd "$OUT"
sha256sum roboclaw_maquina.txt harness_roboclaw_bug1.cpp harness_roboclaw_bug2.cpp > SHA256SUMS.txt

echo
echo "===================================================="
echo "PRONTO. Arquivos gerados em: $OUT"
echo "  - roboclaw_maquina.txt"
echo "  - harness_roboclaw_bug1.cpp"
echo "  - harness_roboclaw_bug2.cpp"
echo "  - SHA256SUMS.txt"
echo "Me mande esses quatro arquivos na conversa."
echo "===================================================="
cat SHA256SUMS.txt
