#!/usr/bin/env bash
# gerar_log_roboclaw_v2.sh
# Gera o log oficial da verificacao formal do RoboClaw (Bug 1 e Bug 2) + hashes,
# AGORA capturando corretamente a versao do ESBMC dentro do arquivo.
# Salve na MESMA pasta dos dois harness e rode:
#   bash gerar_log_roboclaw_v2.sh
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

# versoes (grep -m1 . pega a primeira linha nao-vazia; 2>&1 pois esbmc escreve em stderr)
ESBMC_VER="$(esbmc --version 2>&1 | grep -m1 . )"
CBMC_VER="$(cbmc  --version 2>&1 | grep -m1 . )"

{
  echo "# RoboClaw - verificacao formal (maquina da Marcela)"
  date
  echo "ESBMC: ${ESBMC_VER}"
  echo "CBMC:  ${CBMC_VER}"
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
echo "PRONTO. Confira a linha 'ESBMC:' no topo do arquivo:"
echo
head -4 "$OUT/roboclaw_maquina.txt"
echo
echo "Arquivos em: $OUT"
echo "Me mande: roboclaw_maquina.txt e SHA256SUMS.txt"
echo "===================================================="
cat SHA256SUMS.txt
