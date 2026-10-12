#!/bin/bash
# Copia os logs do iartes para esta pasta (v4). Rode de dentro de 4_resultados_ATUAL/.
set -euo pipefail
ORIGEM="${ORIGEM:-$HOME/TUDO_FINAL_v2/TUDO_FINAL/3_resultados}"
DESTINO="$(pwd)"
[ -d "$ORIGEM" ] || { echo "ERRO: nao encontrei $ORIGEM (ajuste ORIGEM=...)"; exit 1; }
FALTAM=0
copiar() { # origem destino_nome
  if [ -f "$1" ]; then cp -v "$1" "$DESTINO/$2"; else echo "FALTA ORIGEM: $1"; FALTAM=1; fi
}
copiar "$ORIGEM/rodada1/esbmc_sumd_unwind70.txt" esbmc_sumd_unwind70.txt
copiar "$ORIGEM/rodada1/ubsan_sumd.txt" ubsan_sumd.txt
copiar "$ORIGEM/rodada1/esbmc_sumd_resumo.txt" esbmc_sumd_resumo.txt
copiar "$ORIGEM/rodada1/esbmc_logger.txt" esbmc_logger.txt
copiar "$ORIGEM/rodada1/esbmc_logger_resumo.txt" esbmc_logger_resumo.txt
copiar "$ORIGEM/rodada2/cbmc_iridiumsbd.txt" cbmc_iridiumsbd.txt
copiar "$ORIGEM/rodada2/esbmc_microstrain_v3.txt" esbmc_microstrain_v3.txt
copiar "$ORIGEM/rodada2/esbmc_microstrain_v3_multi.txt" esbmc_microstrain_v3_multi.txt
copiar "$ORIGEM/rodada2/cbmc_microstrain_v3.txt" cbmc_microstrain_v3.txt
copiar "$ORIGEM/rodada2/asan_microstrain_v3.txt" asan_microstrain_v3.txt
copiar "$ORIGEM/esbmc_bmi088_multi.txt" esbmc_bmi088_multi.txt
copiar "$ORIGEM/rodada2/esbmc_bst_multi.txt" esbmc_bst_multi.txt
copiar "$ORIGEM/rodada2/asan_bst_probe_only.txt" asan_bst_probe_only.txt
echo "=== Conferencia ==="
for f in esbmc_sumd_unwind70.txt ubsan_sumd.txt esbmc_logger.txt cbmc_iridiumsbd.txt \
         esbmc_microstrain_v3.txt cbmc_microstrain_v3.txt asan_microstrain_v3.txt \
         esbmc_bmi088_multi.txt esbmc_bst_multi.txt asan_bst_probe_only.txt; do
  if [ -f "$DESTINO/$f" ]; then echo "  OK      $f"; else echo "  FALTA   $f"; FALTAM=1; fi
done
exit $FALTAM
