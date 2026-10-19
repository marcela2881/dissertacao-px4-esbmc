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

# ---------------------------------------------------------------------------
# Fase 1 (GPS e IMU BMI088): funcoes ORIGINAIS extraidas do PX4 por awk (sem
# alteracao) e executadas com AddressSanitizer + UndefinedBehaviorSanitizer.
# ---------------------------------------------------------------------------
mkdir -p build_replay/gen
extrai() {  # $1=arquivo do PX4  $2=assinatura (inicio da linha)  $3=saida
  awk -v ini="$2" 'index($0, ini) == 1 {p = 1} p {print} p && /^}/ {exit}' "$1" > "$3"
  [ -s "$3" ] || { echo "ERRO: nao achei '$2' em $1"; return 1; }
  echo "  extraido: $2 ($(wc -l < "$3") linhas de $1)"
}
SAN="-fsanitize=address,undefined -fno-omit-frame-pointer -g"
B=px4/src/drivers/imu/bosch/bmi088
{
  echo "##### replay gps (fase 1)"
  echo "### data: $(date -Iseconds) | maquina: $(uname -srm) | $(g++ --version | head -1)"
  extrai px4/src/drivers/gps/gps.cpp "void GPS::dumpGpsData(" build_replay/gen/gps_dumpGpsData.inc \
  && g++ -std=c++17 -O0 $SAN -I build_replay replay/replay_gps.cpp -o build_replay/replay_gps \
  && { ./build_replay/replay_gps alcance; echo "--- (esperado a seguir: AddressSanitizer acusa o acesso fora do buffer de 79 bytes)"; \
       ./build_replay/replay_gps injetado 2>&1 | grep -E "===|ERROR: AddressSanitizer|WRITE of size|is located|SUMMARY" | head -6; }
  echo
  echo "##### replay bmi088 (fase 1)"
  extrai $B/BMI088_Accelerometer.cpp "bool BMI088_Accelerometer::FIFORead(" build_replay/gen/bmi088_FIFORead.inc \
  && extrai $B/BMI088_Accelerometer.cpp "void BMI088_Accelerometer::UpdateTemperature(" build_replay/gen/bmi088_UpdateTemperature.inc \
  && g++ -std=c++17 -O0 $SAN -I build_replay -I $B replay/replay_bmi088.cpp -o build_replay/replay_bmi088 \
  && { ./build_replay/replay_bmi088 temperatura; ./build_replay/replay_bmi088 fifo; \
       ./build_replay/replay_bmi088 fifo_aleatorio 2>&1 | grep -E "===|maior|ERROR: AddressSanitizer|READ of size|WRITE of size|is located|SUMMARY|#[0-9] .*FIFORead" | head -10; }
} 2>&1 | tee "$out/replay_fase1.log"
echo "(logs em: $out)"
