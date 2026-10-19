#!/usr/bin/env bash
# Reproduz TODA a evidência do reporte da RoboClaw (R1 e R3), em 3 etapas:
#   1) ESBMC 8.5 no código ORIGINAL do PX4: R1 (FAILED), R2 (SUCCESSFUL, faixa normal) e R3 (FAILED)
#   2) replay: g++ executa o Roboclaw.cpp ORIGINAL com os valores do contraexemplo
#   3) reproduções mínimas (sem PX4), para anexar à issue
# Saída na tela e em ../logs/<data>_<máquina>_reporte_roboclaw/
# Uso (dentro de ~/verificacoes):  ./reporte_roboclaw/rodar_reporte.sh
set -uo pipefail
cd "$(dirname "$0")/.."
raiz="$(pwd)"
out="logs/$(date +%Y-%m-%d)_$(hostname -s)_reporte_roboclaw"; mkdir -p "$out" build_replay
command -v g++ >/dev/null || { echo "ERRO: g++ não encontrado (sudo apt install -y g++)"; exit 2; }

echo "##### 1/3 ESBMC: R1, R2 e R3 (1 execução cada, com asserções de desdobramento)"
SO_TESTES="1 2 3" ./rodar_modulo.sh roboclaw rapido 2>&1 | tee "$out/1_esbmc.txt" | tail -6
# contraexemplo do R1 (valores de outputs[] e o byte enviado)
ult=$(ls -td logs/*_esbmc*_rapido | head -1)
grep -a -A40 "Counterexample" "$ult"/roboclaw_T1_ua_run1.log 2>/dev/null | grep -aE "outputs|byte|g_drive_byte|Violated|assert" | head -20 > "$out/1_contraexemplo_R1.txt"
echo "(contraexemplo do R1 em $out/1_contraexemplo_R1.txt)"

echo; echo "##### 2/3 Replay com o Roboclaw.cpp ORIGINAL"
{
  echo "### data: $(date -Iseconds) | maquina: $(uname -srm) | $(g++ --version | head -1)"
  echo "### px4 commit: $(cat px4/.px4_commit 2>/dev/null || echo desconhecido)"
  g++ -std=c++17 -O0 -I stubs -I px4/src/drivers/roboclaw replay/replay_roboclaw.cpp -o build_replay/replay_roboclaw -lm \
    && ./build_replay/replay_roboclaw
} 2>&1 | tee "$out/2_replay.txt"

echo; echo "##### 3/3 Reproduções mínimas (sem PX4)"
for m in minimo_r1 minimo_r3; do
  echo "--- $m"
  g++ -std=c++17 "reporte_roboclaw/$m.cpp" -o "build_replay/$m" -lm && "./build_replay/$m"
done 2>&1 | tee "$out/3_minimos.txt"

echo; echo "(tudo salvo em: $raiz/$out)"
