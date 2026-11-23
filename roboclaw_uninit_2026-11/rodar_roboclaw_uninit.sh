#!/usr/bin/env bash
# Reproduz com Valgrind (e MSan, se houver clang) o uso de memoria nao inicializada do RoboClaw.
# Uso (na raiz do repositorio):  bash roboclaw_uninit_2026-11/rodar_roboclaw_uninit.sh
# Requer: g++ e valgrind  (sudo apt install -y g++ valgrind). clang++ e opcional (MSan).
set -u
cd "$(dirname "$0")/.."
D=roboclaw_uninit_2026-11; OUT=$D/logs; mkdir -p "$OUT"
{ date -u +%FT%TZ; hostname; uname -a; g++ --version | head -1; valgrind --version; command -v clang++ >/dev/null && clang++ --version | head -1; } > "$OUT/ambiente.txt" 2>&1
g++ -g -O0 -std=c++17 $D/reproducao_roboclaw_uninit.cpp -o "$OUT/repro_original" || exit 1
g++ -g -O0 -std=c++17 -DCORRECAO $D/reproducao_roboclaw_uninit.cpp -o "$OUT/repro_corrigido" || exit 1
res() { printf '%-34s | %s\n' "$1" "$2" | tee -a "$OUT/RESUMO.txt"; }
for v in original corrigido; do
  lg="$OUT/valgrind_$v.log"
  timeout 60 valgrind --track-origins=yes --error-exitcode=9 "$OUT/repro_$v" < /dev/null > "$lg" 2>&1; rc=$?
  n=$(grep -c 'uninitialised' "$lg")
  res "Valgrind, $v" "exit=$rc | linhas com 'uninitialised'=$n"
done
# demonstracao do mecanismo: memoria 'suja' (0x7f) e entrada padrao sem dados -> select pode bloquear (como a fila na placa)
for v in original corrigido; do
  lg="$OUT/valgrind_sujo_$v.log"
  ( sleep 25 | timeout 15 valgrind --malloc-fill=0x7f --track-origins=yes "$OUT/repro_$v" > "$lg" 2>&1 ); rc=$?
  res "Valgrind memoria suja, $v" "exit=$rc (124 = ficou bloqueado ate o limite de 15 s)"
done
if command -v clang++ >/dev/null 2>&1; then
  clang++ -g -O0 -std=c++17 -fsanitize=memory -fno-omit-frame-pointer $D/reproducao_roboclaw_uninit.cpp -o "$OUT/repro_msan_original" 2> "$OUT/msan_compilacao.log" && {
    timeout 60 "$OUT/repro_msan_original" < /dev/null > "$OUT/msan_original.log" 2>&1; res "MSan, original" "exit=$? | $(grep -c 'use-of-uninitialized' "$OUT/msan_original.log") ocorrencia(s) de use-of-uninitialized-value"; }
else
  res "MSan" "clang++ nao instalado; pulado"
fi
echo; echo "Empacote:  tar czf roboclaw_uninit.tgz roboclaw_uninit_2026-11/logs"
