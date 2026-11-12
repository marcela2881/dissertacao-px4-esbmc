#!/usr/bin/env bash
# Reexecucao do guia_reproducao.md (v4) no ambiente da nuvem (Claude), 12/10/2026.
set -u
P="$1"; OUT="$2"; mkdir -p "$OUT"
E=/opt/tools/esbmc-8.1/bin/esbmc
H1=$P/1_harnesses/ANTIGO/rodada1; H2=$P/1_harnesses/ANTIGO/rodada2; HA=$P/1_harnesses/ATUAL
R1=$P/2_replays/ANTIGO/rodada1; R2=$P/2_replays/ANTIGO/rodada2; RA=$P/2_replays/ATUAL
hdr(){ echo "# $(date -u +%FT%TZ) host=$(hostname) cmd: $*"; }
run(){ local n=$1; shift; { hdr "$@"; timeout 1800 "$@"; echo "# exit: $?"; } > "$OUT/$n.txt" 2>&1; }
cd "$H1"
run esbmc_c2_external   $E test_external_checks_events.cpp --incremental-bmc --overflow-check --timeout 120
run esbmc_c3_ft7        $E test_ft7_technologies.cpp --incremental-bmc --overflow-check --timeout 120
run esbmc_c4_sumd       $E test_sumd_decode.cpp --unwind 70 --overflow-check
run esbmc_c5_logger     $E test_logger_write_info_multiple.cpp --incremental-bmc --max-k-step 320 --overflow-check --timeout 600
run cbmc_c2_external    cbmc test_external_checks_events.cpp --unwind 10 --bounds-check --signed-overflow-check --unsigned-overflow-check
run cbmc_c3_ft7         cbmc test_ft7_technologies.cpp --unwind 25 --bounds-check --signed-overflow-check --unsigned-overflow-check
run cbmc_c4_sumd        cbmc test_sumd_decode.cpp --unwind 70 --bounds-check --signed-overflow-check --unsigned-overflow-check
cd "$R1"
for x in "c2 replay_external_checks" "c3 replay_ft7"; do set -- $x; g++ -fsanitize=address -g -o /tmp/r_$1 $2.cpp > "$OUT/asan_$1_build.txt" 2>&1; { hdr "./r_$1 ($2.cpp, ASan)"; /tmp/r_$1; echo "# exit: $?"; } > "$OUT/asan_$1.txt" 2>&1; done
g++ -fsanitize=address -g -o /tmp/r_c4a replay_sumd.cpp; { hdr "replay_sumd ASan"; /tmp/r_c4a; echo "# exit: $?"; } > "$OUT/asan_c4_sumd.txt" 2>&1
g++ -fsanitize=undefined -fno-sanitize-recover=all -g -O0 -o /tmp/r_c4u replay_sumd.cpp; { hdr "replay_sumd UBSan"; /tmp/r_c4u; echo "# exit: $?"; } > "$OUT/ubsan_c4_sumd.txt" 2>&1
cd "$H2"
run esbmc_c6_iridium    $E test_iridiumsbd_write.cpp --incremental-bmc --overflow-check --no-library --timeout 120
run esbmc_c7_bst        $E test_bst_dev_name.cpp --incremental-bmc --overflow-check --timeout 120
run cbmc_c6_iridium     cbmc test_iridiumsbd_write.cpp --unwind 400 -D'ssize_t=long' --bounds-check --signed-overflow-check --unsigned-overflow-check
run cbmc_c7_bst         cbmc test_bst_dev_name.cpp --unwind 10 --bounds-check --signed-overflow-check --unsigned-overflow-check
cd "$HA"
run esbmc_c8_microstrain_v3 $E test_microstrain_port_v3.cpp --unwind 200 --overflow-check --timeout 300
run cbmc_c8_microstrain_v3  cbmc test_microstrain_port_v3.cpp --unwind 200 --bounds-check --signed-overflow-check --unsigned-overflow-check
cd "$R2"; g++ -fsanitize=address -g -o /tmp/r_c6 replay_iridiumsbd.cpp; { hdr "replay_iridiumsbd ASan"; /tmp/r_c6; echo "# exit: $?"; } > "$OUT/asan_c6_iridium.txt" 2>&1
cd "$RA"; g++ -fsanitize=address -g -o /tmp/r_c7 replay_bst_probe_only.cpp; { hdr "replay_bst_probe_only ASan"; /tmp/r_c7; echo "# exit: $?"; } > "$OUT/asan_c7_bst.txt" 2>&1
g++ -fsanitize=address -g -O0 -o /tmp/r_c8 replay_microstrain_v3.cpp; { hdr "replay_microstrain_v3 ASan"; /tmp/r_c8; echo "# exit: $?"; } > "$OUT/asan_c8_microstrain_v3.txt" 2>&1
echo FIM
