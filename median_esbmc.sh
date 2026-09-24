#!/usr/bin/env bash
set -u
RUNS=3
LOGDIR="medianas_logs"
mkdir -p "$LOGDIR"
[ "$#" -lt 2 ] && { echo "uso: $0 <label> <comando esbmc completo>" >&2; exit 1; }
label="$1"; shift
times=()
for i in $(seq 1 "$RUNS"); do
  log="$LOGDIR/${label}_run${i}.log"
  "$@" > "$log" 2>&1
  t=$(grep -aoiE "BMC program time: [0-9.]+" "$log" | grep -aoE "[0-9.]+" | tail -1)
  [ -z "$t" ] && t="NA"
  times+=("$t")
  v=$(grep -aoiE "VERIFICATION (SUCCESSFUL|FAILED)" "$log" | tail -1)
  printf '  run %d: %ss   [%s]\n' "$i" "$t" "${v:-sem veredito}"
done
med=$(printf '%s\n' "${times[@]}" | grep -v NA | sort -n | awk '{v[NR]=$1} END{if(NR)printf "%.3f", v[int((NR+1)/2)]; else printf "NA"}')
printf '  >> %s: mediana = %ss\n' "$label" "$med"
