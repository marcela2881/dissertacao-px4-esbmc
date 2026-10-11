#!/usr/bin/env bash
# Itens 4.1 e 4.2 do dossie: verificacao do patch do GPS (PR #27576) e do
# driver de producao com o buffer real de 79 bytes.
#
# Uso:  ./rodar_verificacao_gps.sh <maquina> [esbmc] [cbmc] [timeout_s] [execucoes]
#   no iartes:  ./rodar_verificacao_gps.sh iartes ~/esbmc-v8.1/bin/esbmc cbmc 1800 3
#
# Gera logs/<maquina>/<grupo>_<harness>_<ferramenta>[_uN]_rK.log, cada um com
# cabecalho (data, maquina, CPU, versao, comando) e tempo de parede no fim.
# Timeout (exit 124) = SEM VEREDITO; nunca contar como "passou".
# Variavel opcional SO=cbmc|esbmc roda so uma das ferramentas.
set -u
MAQ="${1:?informe o nome da maquina}"; ESBMC="${2:-esbmc}"; CBMC="${3:-cbmc}"
TO="${4:-1800}"; RUNS="${5:-1}"
cd "$(dirname "$0")"; OUT="logs/$MAQ"; mkdir -p "$OUT"

run() { # grupo ferramenta binario arquivo flags...
  local g="$1" tool="$2" bin="$3" f="$4"; shift 4
  [ -n "${SO:-}" ] && [ "$SO" != "$tool" ] && return 0   # SO=cbmc ou SO=esbmc filtra
  local tag="${g}_$(basename "${f%.cpp}")_${tool}"
  case " $* " in *" --unwind "*) tag="${tag}_u$(echo " $* " | sed -E 's/.* --unwind ([0-9]+) .*/\1/')";; esac
  for k in $(seq 1 "$RUNS"); do
    local lg="$OUT/${tag}_r${k}.log"
    { echo "# data: $(date -u +%FT%TZ)  maquina: $MAQ  host: $(hostname)"
      echo "# cpu:$(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2-)  nucleos: $(nproc)  mem: $(free -g | awk '/Mem/{print $2}') GB"
      echo "# versao: $("$bin" --version 2>&1 | head -1)"
      echo "# comando: $bin $f $*  (timeout ${TO}s)"; } > "$lg"
    local t0; t0=$(date +%s.%N)
    timeout "$TO" "$bin" "$f" "$@" >> "$lg" 2>&1; local rc=$?
    local dt; dt=$(echo "$(date +%s.%N) - $t0" | bc)
    echo "# exit: $rc$([ $rc -eq 124 ] && echo '  (TIMEOUT: sem veredito)')  parede: ${dt} s" >> "$lg"
    printf "%-55s %-12s %s\n" "$(basename "$lg")" \
      "$(grep -aoE 'VERIFICATION (SUCCESSFUL|FAILED)' "$lg" | tail -1 | awk '{print $2}')" "$(tail -1 "$lg")"
  done
}

# --- 4.2: producao (79 bytes), prova indutiva; sem laco => sem desenrolamento
for f in test_gps_prod_inducao.cpp test_gps_prod_sem_invariante.cpp; do
  run prod cbmc  "$CBMC"  "$f" --pointer-check --bounds-check
  run prod esbmc "$ESBMC" "$f" --default-solver boolector
done

# --- 4.1 (a): ESBMC no MESMO harness da Tabela II (test_gps_driver.cpp),
#     original x patch do PR x patch v2. Flags = comando padrao do artigo.
ESBMC_T2=(--unwind 10 --no-unwinding-assertions --overflow-check --default-solver boolector)
run t2 esbmc "$ESBMC" ../resultados/GPS/test_gps_driver.cpp "${ESBMC_T2[@]}"
run t2 esbmc "$ESBMC" test_gps_driver_patch_pr.cpp "${ESBMC_T2[@]}"
run t2 esbmc "$ESBMC" test_gps_driver_patch_v2.cpp "${ESBMC_T2[@]}"

# --- 4.1 (b): CBMC nos harnesses de janela justa (len >= 190), mesmas flags de
#     cbmc/seeded_gps/COMANDO.txt; unwind 2, 3, 5 (com 10 nao termina: artigo).
for u in 2 3 5; do
  for f in test_gps_fair_sem_patch.cpp test_gps_patch_fair.cpp test_gps_patch_v2.cpp; do
    run fair cbmc "$CBMC" "$f" --unwind "$u" --no-unwinding-assertions --bounds-check \
      --pointer-check --signed-overflow-check --unsigned-overflow-check
  done
done
