#!/usr/bin/env bash
# Roda os 10 testes seeded com Boolector e Z3, 3 runs cada, e reporta mediana.
# Uso: ./rodar_seeded.sh /caminho/para/esbmc
set -u
ESBMC="${1:-esbmc}"
LOG=seeded_logs; mkdir -p "$LOG"
tests="test_size_50 test_size_100 test_size_200 test_size_500 test_size_1000 test_size_2000 test_loop_unwind10 test_loop_corrupted_start test_uint16_width test_double_guard_unprotected"
med() { printf '%s\n' "$@" | grep -v NA | sort -n | awk '{v[NR]=$1} END{if(NR)printf "%.3f",v[int((NR+1)/2)];else printf "NA"}'; }
run3() {
  local file="$1" solver="$2"; local t=()
  for i in 1 2 3; do
    local lg="$LOG/${file}_${solver}_r${i}.log"
    "$ESBMC" "${file}.c" --unwind 10 --no-unwinding-assertions --overflow-check --default-solver "$solver" --timeout 120s > "$lg" 2>&1
    local v=$(grep -aoiE "VERIFICATION (SUCCESSFUL|FAILED)" "$lg" | tail -1)
    local x=$(grep -aoiE "BMC program time: [0-9.]+" "$lg" | grep -aoE "[0-9.]+" | tail -1)
    [ -z "$x" ] && x=$(grep -aoiE "Runtime decision procedure: [0-9.]+" "$lg" | grep -aoE "[0-9.]+" | tail -1)
    [ -z "$x" ] && x="NA"
    t+=("$x"); local vv="$v"
  done
  echo "$(med "${t[@]}")|$vv"
}
printf "%-32s | %-22s | %-22s\n" "arquivo" "Boolector (med|verdict)" "Z3 (med|verdict)"
for f in $tests; do
  b=$(run3 "$f" boolector); z=$(run3 "$f" z3)
  printf "%-32s | %-22s | %-22s\n" "$f" "$b" "$z"
done
