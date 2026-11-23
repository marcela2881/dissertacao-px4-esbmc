#!/usr/bin/env bash
# Verifica as guardas do FIFO do BMI088 (pedido do orientador: "verificar formalmente as tres guardas").
# 8 variantes (com/sem G1, G2, G3) x 2 propriedades x 2 ferramentas (ESBMC e CBMC).
# Uso (na raiz do repositorio):  bash fifo_guardas_2026-11/rodar_fifo_guardas.sh
set -u
cd "$(dirname "$0")/.."
OUT=fifo_guardas_2026-11/logs; mkdir -p "$OUT"
{ date -u +%FT%TZ; hostname; esbmc --version | head -1; cbmc --version | head -1; } > "$OUT/ambiente.txt" 2>&1
cat > "$OUT/p.c" <<'EOC'
#include <assert.h>
int nondet_int(void);
int main(void){ int x = nondet_int(); assert(x != 5); return 0; }
EOC
SOLVER=boolector; esbmc "$OUT/p.c" --default-solver boolector 2>&1 | grep -q 'has not been built' && SOLVER=z3
echo "solver do ESBMC: $SOLVER" | tee -a "$OUT/ambiente.txt"
printf '%-12s %-7s | %-22s | %-22s\n' "variante" "prop" "ESBMC" "CBMC" | tee "$OUT/MATRIZ.txt"
for v in "" "SEM_G1" "SEM_G2" "SEM_G3" "SEM_G1 SEM_G2" "SEM_G1 SEM_G3" "SEM_G2 SEM_G3" "SEM_G1 SEM_G2 SEM_G3"; do
  nome=${v:-"G1+G2+G3"}; defs=""; for g in $v; do defs="$defs -D$g"; done
  for p in 1 2; do
    base="$OUT/$(echo "$nome" | tr ' +' '__')_prop$p"
    esbmc fifo_guardas_2026-11/test_fifo_guardas.c $defs -DPROP=$p --unwind 3 --overflow-check --default-solver $SOLVER > "$base.esbmc.log" 2>&1
    cbmc fifo_guardas_2026-11/test_fifo_guardas.c $defs -DPROP=$p -DUSE_CBMC --unwind 3 --unsigned-overflow-check --signed-overflow-check --bounds-check --pointer-check > "$base.cbmc.log" 2>&1
    ve=$(grep -aoE 'VERIFICATION (SUCCESSFUL|FAILED)' "$base.esbmc.log" | tail -1); vc=$(grep -aoE 'VERIFICATION (SUCCESSFUL|FAILED)' "$base.cbmc.log" | tail -1)
    printf '%-12s %-7s | %-22s | %-22s\n' "$nome" "prop$p" "${ve:-SEM VEREDITO}" "${vc:-SEM VEREDITO}" | tee -a "$OUT/MATRIZ.txt"
  done
done
