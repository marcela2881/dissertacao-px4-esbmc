#!/usr/bin/env bash
# Gera a evidência (a) de teste unitário e (b) da inspeção por leitura do código.
# Uso: bash gerar_evidencia.sh CAMINHO_DA_ARVORE_DO_PX4   (a pasta que contém src/ e msg/)
# Árvore usada em 23/11/2026: commit 8be2590c15 (o mesmo da campanha).
set -u
P="${1:?informe o caminho da árvore do PX4}"; cd "$P" || exit 1
echo "### (a) Arquivos de teste que citam sumd, batmon ou roboclaw (sem diferenciar maiúsculas)"
find src -type f \( -iname '*test*' -o -path '*/test/*' -o -path '*/tests/*' \) \
  \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.c' -o -name 'CMakeLists.txt' \) -print0 \
  | xargs -0 grep -l -i -E 'sumd|batmon|roboclaw' 2>/dev/null; echo "(fim da lista)"
echo; echo "### (a.1) sumd: onde sumd_decode e chamado"
grep -rn --include=*.cpp --include=*.hpp --include=*.h --include=*.c -E '\bsumd_decode\b' src
echo; echo "### (a.2) sumd: o teste RCTest::sumdTest (trecho)"
n=$(grep -n 'bool RCTest::sumdTest' src/lib/rc/rc_tests/RCTest.cpp | cut -d: -f1)
sed -n "${n},$((n+70))p" src/lib/rc/rc_tests/RCTest.cpp
echo; echo "### (a.3) batmon: ocorrências da classe Batmon fora de src/drivers/smart_battery/batmon"
grep -rn --include=*.cpp --include=*.hpp --include=*.h -E '\bBatmon\b' src | grep -v 'src/drivers/smart_battery/batmon/' ; echo "(nenhuma se vazio)"
echo; echo "### (a.4) roboclaw: ocorrências de Roboclaw fora de src/drivers/roboclaw"
grep -rn --include=*.cpp --include=*.hpp --include=*.h -E '\bRoboclaw\b' src | grep -v 'src/drivers/roboclaw/' ; echo "(nenhuma se vazio)"
echo; echo "### (b) STATUSTEXT: MavlinkStatustextHandler.cpp, linhas com strlen/strncpy/sizeof"
grep -n -E 'strlen|strncpy|sizeof' src/modules/mavlink/MavlinkStatustextHandler.cpp
echo; echo "### (b.1) tamanho do buffer de destino"
grep -n 'text\[' msg/LogMessage.msg; grep -n '_log_msg{}' src/modules/mavlink/MavlinkStatustextHandler.hpp
echo; echo "### (b.2) RC Update: memcpy e limite de canais"
grep -n -E 'memcpy|channel_count_limited|static_assert' src/modules/rc_update/rc_update.cpp
grep -n 'RC_MAX_CHAN_COUNT{' src/modules/rc_update/rc_update.h; grep -n 'uint16\[18\] values' msg/InputRc.msg
