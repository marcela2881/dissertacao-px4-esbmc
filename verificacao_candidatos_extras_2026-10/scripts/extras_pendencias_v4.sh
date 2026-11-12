#!/usr/bin/env bash
# Itens pendentes do CORRECOES_v4 executados no ambiente da nuvem (Claude), 12/10/2026.
set -u
P="$1"; N="$2"; OUT="$3"; mkdir -p "$OUT"; B=$(mktemp -d)
E=/opt/tools/esbmc-8.1/bin/esbmc
hdr(){ echo "# $(date -u +%FT%TZ) host=$(hostname) cwd=$(pwd)"; echo "# cmd: $*"; }
run(){ local n=$1; shift; { hdr "$@"; timeout 1800 "$@"; echo "# exit: $?"; } > "$OUT/$n.txt" 2>&1; }
# (a) iridium ESBMC: comando do guia (linha 148) falha no parser; repetir com -D ssize_t=long
cd $P/1_harnesses/ANTIGO/rodada2
run esbmc_c6_iridium_ssize $E test_iridiumsbd_write.cpp -D'ssize_t=long' --incremental-bmc --overflow-check --no-library --timeout 120
# (b) candidato 1: ESBMC e CBMC nos 4 harnesses, com e sem unwinding assertions
cd $P/4_resultados_ATUAL/cand1_reexecucao_claude
for h in orig_72 orig_168 inv_72 inv_168; do
  run cand1_esbmc_$h $E $h.cpp --unwind 10 --multi-property --overflow-check --timeout 300
  run cand1_cbmc_$h cbmc $h.cpp --unwind 10 --unwinding-assertions --bounds-check --pointer-check --signed-overflow-check --unsigned-overflow-check
done
# (c) externalChecks: --generate-testcase
cd $P/1_harnesses/ANTIGO/rodada1
mkdir -p $B/tc && cp test_external_checks_events.cpp $B/tc/ && cd $B/tc
run esbmc_c2_external_testcase $E test_external_checks_events.cpp --incremental-bmc --overflow-check --timeout 120 --generate-testcase
ls -la $B/tc >> "$OUT/esbmc_c2_external_testcase.txt"; for x in $B/tc/*.xml; do [ -f "$x" ] && { echo "== $x"; cat "$x"; cp "$x" "$OUT/"; } >> "$OUT/esbmc_c2_external_testcase.txt"; done
# (d) bst e sumd: clang -fsanitize=local-bounds (e bounds) 
cd $P/2_replays/ATUAL
clang++ -g -O0 -fsanitize=address -fsanitize=local-bounds -o $B/bst_lb replay_bst_probe_only.cpp > "$OUT/clang_localbounds_c7_bst_build.txt" 2>&1
{ hdr "clang++ -fsanitize=address -fsanitize=local-bounds replay_bst_probe_only.cpp && ./bst_lb"; $B/bst_lb; echo "# exit: $?"; } > "$OUT/clang_localbounds_c7_bst.txt" 2>&1
cd $P/2_replays/ANTIGO/rodada1
clang++ -g -O0 -fsanitize=local-bounds -o $B/sumd_lb replay_sumd.cpp > "$OUT/clang_localbounds_c4_sumd_build.txt" 2>&1
{ hdr "clang++ -fsanitize=local-bounds replay_sumd.cpp && ./sumd_lb"; $B/sumd_lb; echo "# exit: $?"; } > "$OUT/clang_localbounds_c4_sumd.txt" 2>&1
# (e) ft7: replay que chama collect_call(); original x corrigido upstream (#28600)
cd $N
for v in ORIG FIXED; do
  D=""; [ $v = FIXED ] && D="-DFIXED"
  g++ -g -O0 $D -fsanitize=undefined -fsanitize=bounds-strict -fno-sanitize-recover=all -o $B/ft7_ub_$v replay_ft7_collect.cpp
  g++ -g -O0 $D -fsanitize=address -o $B/ft7_as_$v replay_ft7_collect.cpp
  for c in 1 2; do
    { hdr "g++ $D -fsanitize=undefined,bounds-strict replay_ft7_collect.cpp && ./ft7 $c"; $B/ft7_ub_$v $c; echo "# exit: $?"; } > "$OUT/ubsan_c3_ft7_collect_${v}_cen$c.txt" 2>&1
    { hdr "g++ $D -fsanitize=address replay_ft7_collect.cpp && ./ft7 $c"; $B/ft7_as_$v $c; echo "# exit: $?"; } > "$OUT/asan_c3_ft7_collect_${v}_cen$c.txt" 2>&1
  done
done
echo FIM
