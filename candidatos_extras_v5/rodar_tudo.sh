#!/usr/bin/env bash
# rodar_tudo.sh — pacote v5 (12/10/2026)
#
# Roda TODOS os comandos do pacote e grava um log por comando, com cabecalho
# (data, maquina, pasta, comando exato, versoes) e codigo de saida.
# No fim grava RESUMO.md (esperado x obtido) e preenche as issues.
#
# Uso:
#   ./rodar_tudo.sh                      # resultados em ./resultados_<data>
#   ./rodar_tudo.sh <pasta_de_saida>
#
# Variaveis opcionais:
#   ESBMC=/caminho/esbmc  CBMC=/caminho/cbmc  CXX=clang++|g++
#   SEM_PX4=1   pula a conferencia contra o codigo do PX4 (precisa de git e internet)
#   PX4_CLONE=/caminho   onde fica o clone do PX4 (padrao: ~/px4_conferencia)
#
# Codigo de saida: 0 = tudo como esperado; 2 = algum resultado diferente do
# esperado (ver RESUMO.md); 1 = erro de ambiente (ferramenta faltando etc.).

set -uo pipefail

AQUI="$(cd "$(dirname "$0")" && pwd)"
OUT="${1:-$AQUI/resultados_$(date +%Y%m%d_%H%M)}"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"

ESBMC="${ESBMC:-esbmc}"
CBMC="${CBMC:-cbmc}"
# escolhe um compilador cujo ASan e UBSan realmente linkam (nem todo clang++
# instalado traz as bibliotecas dos sanitizadores)
sanitizadores_ok() { # <compilador>
	command -v "$1" >/dev/null 2>&1 || return 1
	local d; d=$(mktemp -d)
	printf 'int main(){return 0;}\n' > "$d/t.cpp"
	"$1" -fsanitize=address -o "$d/a" "$d/t.cpp" >/dev/null 2>&1 && "$d/a" >/dev/null 2>&1 &&
	"$1" -fsanitize=undefined -o "$d/u" "$d/t.cpp" >/dev/null 2>&1 && "$d/u" >/dev/null 2>&1
	local r=$?; rm -rf "$d"; return $r
}
if [ -z "${CXX:-}" ]; then
	for c in clang++ g++ c++; do
		if sanitizadores_ok "$c"; then CXX=$c; break; fi
	done
	if [ -z "${CXX:-}" ]; then
		echo "ERRO: nenhum compilador (clang++, g++, c++) linka ASan e UBSan nesta maquina." >&2
		exit 1
	fi
elif ! sanitizadores_ok "$CXX"; then
	echo "ERRO: $CXX nao consegue compilar com -fsanitize=address/undefined." >&2
	exit 1
fi

falta=0
for t in "$ESBMC" "$CBMC" "$CXX"; do
	if ! command -v "$t" >/dev/null 2>&1; then
		echo "ERRO: '$t' nao encontrado no PATH. Ver LEIA-ME.md, secao Ambiente." >&2
		falta=1
	fi
done
[ "$falta" = 0 ] || exit 1

# timeout portavel (macOS nao tem 'timeout'; com Homebrew coreutils e 'gtimeout')
TO=""
if command -v timeout >/dev/null 2>&1; then TO="timeout 3600";
elif command -v gtimeout >/dev/null 2>&1; then TO="gtimeout 3600"; fi

agora() { date -u +%Y-%m-%dT%H:%M:%SZ; }

# ---------------------------------------------------------------- ambiente
{
	echo "data:      $(agora)"
	echo "maquina:   $(hostname)"
	echo "sistema:   $(uname -a)"
	echo "esbmc:     $(command -v "$ESBMC")"
	"$ESBMC" --version 2>&1 | head -3
	echo "cbmc:      $(command -v "$CBMC")"
	"$CBMC" --version 2>&1 | head -1
	echo "compilador: $(command -v "$CXX")"
	"$CXX" --version 2>&1 | head -1
} > "$OUT/ambiente.txt"
ESBMC_V=$("$ESBMC" --version 2>&1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)
CBMC_V=$("$CBMC" --version 2>&1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)
CXX_V=$("$CXX" --version 2>&1 | head -1)

# ---------------------------------------------------------------- funcoes
RES="$OUT/.resultados.tsv"
: > "$RES"

# copia as fontes do candidato para a saida e entra nela
prepara() { # <pasta_do_candidato>
	CAND="$1"
	mkdir -p "$OUT/$CAND/fontes"
	cp -R "$AQUI/candidatos/$CAND/." "$OUT/$CAND/fontes/"
	cd "$OUT/$CAND/fontes" || exit 1
}

veredito() { # <arquivo_de_log> -> FAILED | SUCCESSFUL | ASAN | UBSAN | ERRO_COMPILACAO | NADA | OUTRO
	local f="$1"
	if grep -q "ERRO_COMPILACAO" "$f"; then echo ERRO_COMPILACAO
	elif grep -q "VERIFICATION FAILED" "$f"; then echo FAILED
	elif grep -q "VERIFICATION SUCCESSFUL" "$f"; then echo SUCCESSFUL
	elif grep -q "ERROR: AddressSanitizer" "$f"; then echo ASAN
	elif grep -q "runtime error:" "$f"; then echo UBSAN
	elif grep -q "^# codigo de saida: 0$" "$f"; then echo NADA
	else echo OUTRO
	fi
}

# roda um comando e grava o log
roda() { # <nome_do_log> <esperado> <descricao> -- comando...
	local nome="$1" esperado="$2" desc="$3"; shift 4
	local log="$OUT/$CAND/$nome.txt"
	echo "  $CAND/$nome ..."
	{
		echo "# data:     $(agora)"
		echo "# maquina:  $(hostname)"
		echo "# pasta:    $(pwd)"
		echo "# comando:  $*"
		echo "# o que e:  $desc"
		echo "# --------------------------------------------------------------"
		$TO "$@"
		echo "# codigo de saida: $?"
	} > "$log" 2>&1
	local obtido; obtido=$(veredito "$log")
	printf '%s\t%s\t%s\t%s\t%s\n' "$CAND" "$nome" "$esperado" "$obtido" "$desc" >> "$RES"
}

# compila e roda um binario (sanitizador ou nao)
compila_roda() { # <nome_do_log> <esperado> <descricao> <flags> <fonte> -- args...
	local nome="$1" esperado="$2" desc="$3" flags="$4" fonte="$5"; shift 6
	local bin="$OUT/$CAND/fontes/$nome.bin"
	local log="$OUT/$CAND/$nome.txt"
	echo "  $CAND/$nome ..."
	{
		echo "# data:     $(agora)"
		echo "# maquina:  $(hostname)"
		echo "# pasta:    $(pwd)"
		echo "# compilar: $CXX $flags -g -O0 -o $nome.bin $fonte"
		echo "# executar: ./$nome.bin $*"
		echo "# o que e:  $desc"
		echo "# --------------------------------------------------------------"
		# shellcheck disable=SC2086
		if "$CXX" $flags -g -O0 -o "$bin" "$fonte"; then
			"$bin" "$@"
			echo "# codigo de saida: $?"
		else
			echo "ERRO_COMPILACAO"
		fi
	} > "$log" 2>&1
	local obtido; obtido=$(veredito "$log")
	printf '%s\t%s\t%s\t%s\t%s\n' "$CAND" "$nome" "$esperado" "$obtido" "$desc" >> "$RES"
}

# vazamento de memoria nao e o que se testa aqui, e o LeakSanitizer falha
# dentro de conteineres (podman/docker); por isso fica desligado
export ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}"

CHECKS_CBMC="--bounds-check --pointer-check --signed-overflow-check --unsigned-overflow-check"
ASAN="-fsanitize=address -fno-omit-frame-pointer"
UBSAN="-fsanitize=undefined -fno-sanitize-recover=all"

echo "Saida: $OUT"
echo "ESBMC $ESBMC_V | CBMC $CBMC_V | $CXX_V"

# ======================================================== 0. conferencia PX4
if [ "${SEM_PX4:-0}" != 1 ]; then
	echo "0. conferencia contra o codigo do PX4 (git) ..."
	mkdir -p "$OUT/0_px4"
	if bash "$AQUI/conferir_px4.sh" "$OUT/0_px4" "${PX4_CLONE:-$HOME/px4_conferencia}" > "$OUT/0_px4/saida.txt" 2>&1; then
		echo "   ok"
	else
		echo "   FALHOU (ver $OUT/0_px4/saida.txt); seguindo sem ela"
	fi
fi

# ======================================================== 1. Common (retirado)
echo "1. Common.cpp (retirado)"
prepara 1_Common_RETIRADO
for h in orig_72 orig_168 inv_72 inv_168; do
	case $h in orig_*) esp=FAILED; d="premissa antiga idx <= SIZE+4";; *) esp=SUCCESSFUL; d="invariante da guarda idx <= SIZE";; esac
	roda "esbmc_$h" $esp "$d (SIZE ${h#*_}; 168 e o tamanho real)" -- "$ESBMC" $h.cpp --unwind 200 --multi-property --overflow-check
	roda "cbmc_$h"  $esp "$d (SIZE ${h#*_})" -- "$CBMC" $h.cpp --unwind 200 --unwinding-assertions $CHECKS_CBMC
done

# ======================================================== 2. externalChecks
echo "2. externalChecks.cpp (corrigido upstream)"
prepara 2_externalChecks
roda esbmc            FAILED     "comando do guia" -- "$ESBMC" harness.cpp --incremental-bmc --overflow-check --timeout 120 --quiet
roda esbmc_multi      FAILED     "contagem de propriedades, bound fixo (B16)" -- "$ESBMC" harness.cpp --unwind 10 --no-unwinding-assertions --multi-property --overflow-check
roda esbmc_correcao   SUCCESSFUL "harness com a correcao upstream ca1d36476a" -- "$ESBMC" harness.cpp -DCORRECAO_UPSTREAM --unwind 10 --overflow-check
roda cbmc             FAILED     "com trace para concretizar num_events (B6)" -- "$CBMC" harness.cpp --unwind 260 --unwinding-assertions --object-bits 12 $CHECKS_CBMC --trace
roda cbmc_correcao    SUCCESSFUL "harness com a correcao upstream" -- "$CBMC" harness.cpp -DCORRECAO_UPSTREAM --unwind 260 --unwinding-assertions --object-bits 12 $CHECKS_CBMC
N2=$(grep -oE 'reply\.num_events=[0-9]+' "$OUT/$CAND/cbmc.txt" | grep -oE '[0-9]+$' | sort -n | tail -1)
N2="${N2:-9}"
echo "$N2" > "$OUT/$CAND/contraexemplo_num_events.txt"
compila_roda asan ASAN "replay com num_events=$N2 tirado do trace do CBMC" "$ASAN" replay.cpp -- "$N2"

# ======================================================== 3. ft7
echo "3. ft7_technologies.cpp (corrigido upstream)"
prepara 3_ft7
roda esbmc            FAILED     "comando do guia (a 1a propriedade impressa varia com a versao, B20)" -- "$ESBMC" harness.cpp --incremental-bmc --overflow-check --timeout 120 --quiet
roda esbmc_multi      FAILED     "contagem de propriedades, bound fixo (B16)" -- "$ESBMC" harness.cpp --unwind 25 --multi-property --overflow-check
roda esbmc_correcao   SUCCESSFUL "harness com a correcao upstream 3acf7935a0" -- "$ESBMC" harness.cpp -DCORRECAO_UPSTREAM --unwind 25 --overflow-check
roda cbmc             FAILED     "" -- "$CBMC" harness.cpp --unwind 25 --unwinding-assertions $CHECKS_CBMC
roda cbmc_correcao    SUCCESSFUL "harness com a correcao upstream" -- "$CBMC" harness.cpp -DCORRECAO_UPSTREAM --unwind 25 --unwinding-assertions $CHECKS_CBMC
compila_roda ubsan_cenario1          UBSAN "replay chama collect(): 2 leituras de 19 bytes sem \\n" "$UBSAN" replay.cpp -- 1
compila_roda ubsan_cenario2          UBSAN "replay chama collect(): campo speed sem delimitador" "$UBSAN" replay.cpp -- 2
compila_roda ubsan_cenario1_correcao NADA  "mesmo replay com as guardas do upstream" "$UBSAN -DFIXED" replay.cpp -- 1
compila_roda ubsan_cenario2_correcao NADA  "mesmo replay com as guardas do upstream" "$UBSAN -DFIXED" replay.cpp -- 2
compila_roda asan_cenario1           NADA  "ASan nao ve (escrita dentro do objeto); ver _interval corrompido na saida" "$ASAN" replay.cpp -- 1

# ======================================================== 4. sumd
echo "4. sumd.cpp (CONTINUA no PX4)"
prepara 4_sumd
roda esbmc            FAILED "comando do guia" -- "$ESBMC" harness.cpp --unwind 70 --overflow-check
# sem esbmc_multi aqui: com --unwind 70 o --multi-property passa de 15 min (laco de CRC);
# a contagem completa de propriedades vem do CBMC abaixo.
roda cbmc             FAILED "" -- "$CBMC" harness.cpp --unwind 70 --unwinding-assertions $CHECKS_CBMC
compila_roda ubsan_32        UBSAN "sumd_decode() real, quadro de 32 canais" "$UBSAN" replay.cpp -- 32
compila_roda efeito_32       NADA  "sem sanitizador: efeito no canal 32 (max_chan_count=32, como no RCTest)" "" replay.cpp -- 32
compila_roda efeito_18       NADA  "sem sanitizador: max_chan_count=18 (como no RCInput)" "" replay.cpp -- 18
compila_roda asan_32         NADA  "ASan nao ve (escrita dentro do struct)" "$ASAN" replay.cpp -- 32

# ======================================================== 5. logger
echo "5. logger.cpp (sem violacao de memoria)"
prepara 5_logger
roda esbmc            SUCCESSFUL "comando do guia; k=300 e so o laco concreto (A6)" -- "$ESBMC" harness.cpp --incremental-bmc --max-k-step 320 --overflow-check --timeout 600 --quiet
roda esbmc_key_len    FAILED     "key_len livre: so a assercao do truncamento deve falhar (A7)" -- "$ESBMC" harness.cpp -DMODELO_KEY_LEN_NONDET --unwind 10 --multi-property --overflow-check
roda esbmc_sondagem   FAILED     "sondagem do modelo de snprintf (A7, secao 5.4)" -- "$ESBMC" sondagem_snprintf.c
roda cbmc_sondagem    OUTRO      "mesma sondagem no CBMC (so registro)" -- "$CBMC" sondagem_snprintf.c --unwind 12
compila_roda sondagem_libc NADA "mesma sondagem com a libc real (deve imprimir 10)" "-x c" sondagem_snprintf.c --

# ======================================================== 6. IridiumSBD
echo "6. IridiumSBD.cpp (corrigido upstream)"
prepara 6_IridiumSBD
roda esbmc            FAILED     "comando do guia (sem -D ssize_t: harness inclui sys/types.h)" -- "$ESBMC" harness.cpp --incremental-bmc --overflow-check --no-library --timeout 120
roda esbmc_multi      FAILED     "contagem de propriedades, bound fixo (B16)" -- "$ESBMC" harness.cpp --unwind 345 --multi-property --overflow-check --no-library
roda esbmc_correcao   SUCCESSFUL "harness com a correcao upstream 0c903d47d6" -- "$ESBMC" harness.cpp -DCORRECAO_UPSTREAM --unwind 345 --overflow-check --no-library
roda cbmc             FAILED     "" -- "$CBMC" harness.cpp --unwind 345 --unwinding-assertions $CHECKS_CBMC
roda cbmc_correcao    SUCCESSFUL "harness com a correcao upstream" -- "$CBMC" harness.cpp -DCORRECAO_UPSTREAM --unwind 345 --unwinding-assertions $CHECKS_CBMC
B6=$(grep -oE 'second_buflen = [0-9]+' "$OUT/$CAND/esbmc.txt" | tail -1 | grep -oE '[0-9]+$')
B6="${B6:-334}"
P6=$(grep -oE 'payload_len = [0-9]+' "$OUT/$CAND/esbmc.txt" | tail -1 | grep -oE '[0-9]+$')
P6="${P6:-5}"
echo "payload_len=$P6 second_buflen=$B6" > "$OUT/$CAND/contraexemplo.txt"
compila_roda asan ASAN "replay com payload_len=$P6 e second_buflen=$B6 tirados do log do ESBMC (B4)" "$ASAN" replay.cpp -- "$P6" 0 "$B6"

# ======================================================== 7. bst (falso positivo)
echo "7. bst.cpp (falso positivo)"
prepara 7_bst_FALSO_POSITIVO
roda esbmc_sem_guardas FAILED     "harness ANTIGO, sem as guardas de producao (so rastreio)" -- "$ESBMC" harness_sem_guardas.cpp --incremental-bmc --overflow-check --timeout 120 --quiet
roda esbmc_fiel        SUCCESSFUL "harness com as guardas do 8be2590c15" -- "$ESBMC" harness_fiel.cpp --unwind 50 --overflow-check
roda cbmc_fiel         SUCCESSFUL "harness com as guardas do 8be2590c15" -- "$CBMC" harness_fiel.cpp --unwind 50 --unwinding-assertions $CHECKS_CBMC

# ======================================================== 8. MicroStrain
echo "8. MicroStrain.cpp (CONTINUA no PX4)"
prepara 8_MicroStrain
roda esbmc            FAILED "comando do guia" -- "$ESBMC" harness.cpp --unwind 200 --overflow-check --timeout 300
roda esbmc_multi      FAILED "contagem de propriedades, bound fixo (B16)" -- "$ESBMC" harness.cpp --unwind 200 --multi-property --overflow-check
roda cbmc             FAILED "" -- "$CBMC" harness.cpp --unwind 200 --unwinding-assertions $CHECKS_CBMC
compila_roda asan     ASAN   "replay: 128 bytes nao nulos" "$ASAN" replay.cpp --

cd "$OUT" || exit 1

# ======================================================== RESUMO
diverge=0
{
	echo "# Resumo da execucao"
	echo
	echo "- data: $(agora)"
	echo "- maquina: $(hostname)"
	echo "- ESBMC $ESBMC_V, CBMC $CBMC_V, $CXX_V"
	[ -f 0_px4/px4_main_hash.txt ] && echo "- PX4 main conferido: $(cat 0_px4/px4_main_hash.txt) ($(cat 0_px4/px4_main_data.txt))"
	echo
	echo "| Candidato | Log | Esperado | Obtido | Confere | O que e |"
	echo "|---|---|---|---|---|---|"
	while IFS=$'\t' read -r c n e o d; do
		ok="sim"
		if [ "$e" != "$o" ] && [ "$e" != "OUTRO" ]; then ok="**NAO**"; diverge=1; fi
		echo "| $c | $n.txt | $e | $o | $ok | $d |"
	done < "$RES"
	echo
	echo "Contagens de propriedades (ultima linha de contagem de cada log;"
	echo "ESBMC 8.1 imprime 'Properties: ...', ESBMC 8.4 e CBMC imprimem '** N of M ...'):"
	echo
	for f in */*.txt; do
		l=$(grep -E '^\*\* [0-9]+ of [0-9]+|^Properties: ' "$f" | tail -1)
		[ -n "$l" ] && echo "- $f: \`$l\`"
	done
} > RESUMO.md
cat RESUMO.md | grep -E "^\| " | awk -F'|' '{printf "%-22s %-26s %-11s %-11s %s\n",$2,$3,$4,$5,$6}'

# ======================================================== issues preenchidas
mkdir -p "$OUT/issues_prontas"
trecho_esbmc() { awk '/Violated property:/{p=1} p{print} /VERIFICATION FAILED/{if(p) exit}' "$1"; }
trecho_cbmc()  { grep -E 'FAILURE|^\*\* [0-9]+ of' "$1"; }
trecho_san()   { grep -E 'runtime error|ERROR: AddressSanitizer|WRITE of size|READ of size|is located|SUMMARY|^channel|^max_chan' "$1"; }
mkdir -p "$OUT/.trechos"
awk '/Violated property/{exit} /^ *_rxlen = /{l=$0} END{print "Counterexample (last state before the violation):"; print l; print ""}' 4_sumd/esbmc.txt > .trechos/SUMD_ESBMC
trecho_esbmc 4_sumd/esbmc.txt          >> .trechos/SUMD_ESBMC
trecho_cbmc  4_sumd/cbmc.txt           > .trechos/SUMD_CBMC
trecho_san   4_sumd/ubsan_32.txt       > .trechos/SUMD_UBSAN
trecho_san   4_sumd/efeito_32.txt      > .trechos/SUMD_EFEITO
awk '/Violated property/{exit} /max_len = /{l=$0} END{print "Counterexample (last state before the violation):"; print l; print ""}' 8_MicroStrain/esbmc.txt > .trechos/MS_ESBMC
trecho_esbmc 8_MicroStrain/esbmc.txt   >> .trechos/MS_ESBMC
trecho_cbmc  8_MicroStrain/cbmc.txt    > .trechos/MS_CBMC
trecho_san   8_MicroStrain/asan.txt    > .trechos/MS_ASAN
MAIN_HASH=$(cat 0_px4/px4_main_hash.txt 2>/dev/null || echo "<HASH-DO-MAIN>")
MAIN_DATA=$(cat 0_px4/px4_main_data.txt 2>/dev/null || echo "<DATA>")
for t in "$AQUI"/issues_px4/issue_*.md; do
	awk -v dir="$OUT/.trechos" -v h="$MAIN_HASH" -v d="$MAIN_DATA" -v ev="$ESBMC_V" -v cv="$CBMC_V" -v xv="$CXX_V" '
		/^\{\{[A-Z_]+\}\}$/ { n=substr($0,3,length($0)-4); f=dir"/"n; while ((getline l < f) > 0) print l; close(f); next }
		{ gsub(/\{\{PX4_MAIN\}\}/,h); gsub(/\{\{PX4_MAIN_DATA\}\}/,d); gsub(/\{\{ESBMC_V\}\}/,ev); gsub(/\{\{CBMC_V\}\}/,cv); gsub(/\{\{CXX_V\}\}/,xv); print }
	' "$t" > "$OUT/issues_prontas/$(basename "$t")"
done

echo
echo "Resumo: $OUT/RESUMO.md"
echo "Issues preenchidas: $OUT/issues_prontas/"
if [ "$diverge" = 1 ]; then
	echo "ATENCAO: algum resultado diferiu do esperado (ver RESUMO.md)."
	exit 2
fi
echo "Tudo como esperado."
exit 0
