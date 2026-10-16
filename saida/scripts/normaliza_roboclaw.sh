#!/usr/bin/env bash
# Normalização MÍNIMA do Roboclaw.cpp, só para contornar duas limitações do
# frontend C++ do ESBMC (ver saida/ferramenta/). As duas trocas preservam a
# semântica do código; o diff fica salvo junto com os logs da rodada.
#   1) literal definido pelo usuário 11_ms  -> (11 * 1000ULL)   [mesmo valor]
#   2) "const float data" no parâmetro de sendUnsigned7Bit -> "float data"
#      (const de topo em parâmetro não muda o tipo da função; o corpo não altera data)
# Uso: ./normaliza_roboclaw.sh <PX4_ROOT> <pasta_destino>
set -euo pipefail
src="$1/src/drivers/roboclaw/Roboclaw.cpp"; dst="$2"
mkdir -p "$dst"
sed -E -e 's/\b([0-9]+)_ms\b/(\1 * 1000ULL)/g' \
       -e 's/sendUnsigned7Bit\(Command command, const float data\)/sendUnsigned7Bit(Command command, float data)/' \
       "$src" > "$dst/Roboclaw.cpp"
diff -u "$src" "$dst/Roboclaw.cpp" > "$dst/normalizacao.diff" || true
n=$(grep -c '^[-+][^-+]' "$dst/normalizacao.diff" || true)
[ "$n" -eq 4 ] || { echo "ERRO: esperava 2 linhas trocadas (4 linhas no diff), encontrei $n" >&2; exit 1; }
echo "Normalização: $dst/normalizacao.diff"
