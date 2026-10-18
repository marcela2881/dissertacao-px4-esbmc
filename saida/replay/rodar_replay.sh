#!/usr/bin/env bash
# Compila e EXECUTA os testes de reprodução (replay) com o g++ comum, usando os
# arquivos ORIGINAIS do PX4 (pasta ../px4). A saída vai para a tela e para
# ../logs/<data>_<máquina>_replay/.
# Uso: ./replay/rodar_replay.sh
set -uo pipefail
cd "$(dirname "$0")/.."
command -v g++ >/dev/null || { echo "ERRO: g++ não encontrado. Instale com: sudo apt install -y g++"; exit 2; }
[ -d px4/src ] || { echo "ERRO: PX4 não encontrado em ./px4 (rode antes um ./rodar_modulo.sh ... rapido)"; exit 3; }
out="logs/$(date +%Y-%m-%d)_$(hostname -s)_replay"; mkdir -p "$out" build_replay
for alvo in roboclaw:roboclaw pca9685:pca9685_pwm_out; do
  nome=${alvo%%:*}; dir=${alvo##*:}
  echo "##### replay $nome"
  {
    echo "### data: $(date -Iseconds) | maquina: $(uname -srm) | $(g++ --version | head -1)"
    echo "### px4 commit: $(cat px4/.px4_commit 2>/dev/null || echo desconhecido)"
    g++ -std=c++17 -O0 -I stubs -I "px4/src/drivers/$dir" "replay/replay_$nome.cpp" -o "build_replay/replay_$nome" -lm \
      && "./build_replay/replay_$nome"
  } 2>&1 | tee "$out/replay_$nome.log"
  echo
done
echo "(logs em: $out)"
