#!/usr/bin/env bash
# Roda o teste gtest da RoboClaw DENTRO do PX4 (como um desenvolvedor do PX4 faria):
# original (esperado: os 2 testes FALHAM) e com a correção (esperado: os 2 PASSAM).
#
#   ./rodar_gtest_px4.sh [pasta_do_px4]      (padrão: ~/px4_gtest)
#
# Na primeira vez: baixa o PX4 com os submódulos (~2-4 GB) e instala o ambiente de
# compilação do PX4 (Tools/setup/ubuntu.sh, pede a senha do sudo). Depois, só compila.
# Saídas: ~/gtest_roboclaw_original.txt e ~/gtest_roboclaw_corrigido.txt
# (19/10/2026: LINKLIBS inclui mixer_module; sem ele a ligação falhava em MixingOutput)
set -uo pipefail
AQUI="$(cd "$(dirname "$0")" && pwd)"
PATCH="$AQUI/../6_pr/correcao_roboclaw.patch"
D="${1:-$HOME/px4_gtest}"
COMMIT=8be2590c1529517ccd54fbc09ae138402517baf4

if [ ! -d "$D/.git" ]; then
	git clone https://github.com/PX4/PX4-Autopilot.git "$D" || exit 1
fi
cd "$D" || exit 1
git fetch -q origin "$COMMIT" 2>/dev/null; git checkout -q -f "$COMMIT" || exit 1
git submodule update --init --recursive || exit 1

if [ ! -f "$HOME/.px4_setup_ok" ]; then
	bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools && touch "$HOME/.px4_setup_ok" || exit 1
fi

# teste e CMake (só acrescenta; o Roboclaw.cpp não é alterado)
cp "$AQUI/RoboclawTest.cpp" src/drivers/roboclaw/
git checkout -q src/drivers/roboclaw/CMakeLists.txt   # volta ao original antes de acrescentar
cat "$AQUI/CMakeLists_trecho.txt" >> src/drivers/roboclaw/CMakeLists.txt
# o driver não vem ligado no SITL: liga só na configuração de testes
grep -q CONFIG_DRIVERS_ROBOCLAW boards/px4/sitl/test.px4board || echo "CONFIG_DRIVERS_ROBOCLAW=y" >> boards/px4/sitl/test.px4board

echo "##### 1/2 Roboclaw.cpp ORIGINAL (esperado: 2 testes FALHAM)"
make tests TESTFILTER=Roboclaw 2>&1 | tee "$HOME/gtest_roboclaw_original.txt"

echo "##### 2/2 com a CORREÇÃO (esperado: 2 testes PASSAM)"
git apply "$PATCH" || exit 1
make tests TESTFILTER=Roboclaw 2>&1 | tee "$HOME/gtest_roboclaw_corrigido.txt"
git checkout -q src/drivers/roboclaw/Roboclaw.cpp

echo
echo "Resumo:"
grep -hE "^\[  (PASSED|FAILED) |tests passed|tests failed" "$HOME/gtest_roboclaw_original.txt" "$HOME/gtest_roboclaw_corrigido.txt"
