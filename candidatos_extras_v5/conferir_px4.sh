#!/usr/bin/env bash
# conferir_px4.sh — confere os trechos do PX4 que os harnesses copiam.
#
# Uso: ./conferir_px4.sh <pasta_de_saida> [pasta_do_clone_px4]
#
# Baixa o PX4 sem checkout (so o historico, ~200 MB na primeira vez) e,
# para o commit testado (8be2590c15) e para o main do dia, grava:
#   - as linhas exatas de cada trecho copiado nos harnesses;
#   - os commits que alteraram cada arquivo depois do 8be2590c15.
# Nada e compilado; e so leitura do codigo.
set -euo pipefail

OUT="${1:?uso: $0 <pasta_de_saida> [pasta_do_clone_px4]}"
PX4="${2:-$HOME/px4_conferencia}"
BASE=8be2590c15
mkdir -p "$OUT"

if [ ! -d "$PX4/.git" ]; then
	git clone --filter=blob:none --no-checkout https://github.com/PX4/PX4-Autopilot "$PX4"
fi
git -C "$PX4" fetch --quiet origin main
MAIN=$(git -C "$PX4" rev-parse --short=10 origin/main)
MAIN_DATA=$(git -C "$PX4" log -1 --format=%cs origin/main)
echo "$MAIN" > "$OUT/px4_main_hash.txt"
echo "$MAIN_DATA" > "$OUT/px4_main_data.txt"

trecho() { # <rotulo> <arquivo> <inicio> <fim>
	local rot="$1" arq="$2" ini="$3" fim="$4"
	for c in "$BASE" "$MAIN"; do
		echo "=== $rot | $arq | commit $c | linhas $ini-$fim"
		git -C "$PX4" show "$c:$arq" | awk -v a="$ini" -v b="$fim" 'NR>=a && NR<=b {printf "%5d  %s\n", NR, $0}'
		echo
	done
}

procura() { # <rotulo> <arquivo> <regex>
	local rot="$1" arq="$2" re="$3"
	for c in "$BASE" "$MAIN"; do
		echo "=== $rot | $arq | commit $c | grep: $re"
		git -C "$PX4" show "$c:$arq" | grep -n -E "$re" || echo "(nenhuma linha)"
		echo
	done
}

{
	echo "# Conferencia dos trechos copiados nos harnesses"
	echo "# data: $(date -u +%Y-%m-%dT%H:%M:%SZ)  maquina: $(hostname)"
	echo "# commit testado: $BASE ($(git -C "$PX4" log -1 --format=%cs $BASE))"
	echo "# main do dia:    $MAIN ($MAIN_DATA)"
	echo

	procura "1 Common: tamanho do buffer" msg/versioned/Event.msg "ORB_QUEUE_LENGTH"
	procura "1 Common: declaracao do buffer" src/modules/commander/HealthAndArmingChecks/Common.hpp "_event_buffer\[|EventBufferHeader \{"

	procura "2 externalChecks: laco e limite" src/modules/commander/HealthAndArmingChecks/checks/externalChecks.cpp "num_events"

	trecho "3 ft7: laco de collect()" src/drivers/wind_sensor/ft_technologies/ft7_technologies.cpp 158 175
	procura "3 ft7: tamanhos" src/drivers/wind_sensor/ft_technologies/ft7_technologies.hpp "_readbuf\[|_linebuf\[|_raw_"

	trecho "4 sumd: escrita e condicao" src/lib/rc/sumd.cpp 194 210
	procura "4 sumd: tamanho do array" src/lib/rc/sumd.h "SUMD_MAX_CHANNELS|sumd_data\["
	procura "4 sumd: chamadas e max_chan_count" src/drivers/rc_input/RCInput.cpp "sumd_decode\(|RC_INPUT_MAX_CHANNELS"

	procura "5 logger: write_info_multiple" src/modules/logger/logger.cpp "key_len = snprintf|msg_size = sizeof\(msg\)|vlen < \(sizeof"
	procura "5 logger: struct" src/modules/logger/messages.h "key_len;|key_value_str\["

	procura "6 IridiumSBD: guarda e subtracao" src/drivers/telemetry/iridiumsbd/IridiumSBD.cpp "_packet_length -= buflen|_packet_length < 0|space_left|SATCOM_TX_BUF_LEN - _tx_buf_write_idx"

	procura "7 bst: guardas no probe()" src/drivers/telemetry/bst/bst.cpp "dev_name_len|dev_info_reply.length >="

	trecho "8 MicroStrain: construtor" src/drivers/ins/microstrain/MicroStrain.cpp 52 57
	procura "8 MicroStrain: tamanho de _port" src/drivers/ins/microstrain/MicroStrain.hpp "_port\["
	procura "8 MicroStrain: origem do nome da porta" src/drivers/ins/microstrain/MicroStrain.cpp "case 'd'|dev = myoptarg|new MicroStrain"

	echo "=== Commits que alteraram os arquivos depois de $BASE"
	for f in \
		src/modules/commander/HealthAndArmingChecks/checks/externalChecks.cpp \
		src/drivers/wind_sensor/ft_technologies/ft7_technologies.cpp \
		src/lib/rc/sumd.cpp \
		src/modules/logger/logger.cpp \
		src/drivers/telemetry/iridiumsbd/IridiumSBD.cpp \
		src/drivers/telemetry/bst/bst.cpp \
		src/drivers/ins/microstrain/MicroStrain.cpp; do
		echo "--- $f"
		git -C "$PX4" log --format='%h %cs %an %s' "$BASE..origin/main" -- "$f" || true
	done
	echo
	echo "=== Commit que introduziu as guardas do bst"
	git -C "$PX4" log --format='%h %cs %an %s' -S"dev_name_len >= sizeof" -- src/drivers/telemetry/bst/bst.cpp
} > "$OUT/conferencia_px4.txt" 2>&1

echo "conferencia gravada em $OUT/conferencia_px4.txt (main = $MAIN, $MAIN_DATA)"
