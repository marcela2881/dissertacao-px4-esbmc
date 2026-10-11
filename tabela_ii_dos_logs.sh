#!/usr/bin/env bash
# Recalcula a Tabela II do artigo SBESC 2026 a partir dos logs versionados
# (sem rodar o ESBMC). Uso: ./tabela_ii_dos_logs.sh [pasta]   (padrao: medianas_logs)
# Tempo: mediana de "BMC program time"; se ausente (GPS e IMU, que param na
# primeira violacao), mediana de "Runtime decision procedure" (ultima ocorrencia).
set -u
DIR="${1:-medianas_logs}"
med() { printf '%s\n' "$@" | sort -n | awk '{v[NR]=$1} END{printf "%.3f", v[int((NR+1)/2)]}'; }
printf "%-13s %-8s %-24s %-10s %-9s %-5s %-8s %s\n" modulo esbmc solver campo mediana_s VCC veredito "tempos (run1 run2 run3)"
tot=0; totg=0
for m in $(ls "$DIR" | sed -E 's/_run[0-9]+\.log$//' | sort -u); do
  ts=(); campo=BMC
  for f in "$DIR/${m}"_run*.log; do
    t=$(grep -aoE 'BMC program time: [0-9.]+' "$f" | grep -oE '[0-9.]+$' | tail -1)
    if [ -z "$t" ]; then t=$(grep -aoE 'Runtime decision procedure: [0-9.]+' "$f" | grep -oE '[0-9.]+$' | tail -1); campo=RDP; fi
    ts+=("$t")
  done
  f1="$DIR/${m}_run1.log"
  ver=$(grep -m1 -oE 'ESBMC version [0-9.]+' "$f1" | awk '{print $3}')
  sol=$(grep -m1 -aoE 'Solving with solver [A-Za-z0-9]+( [0-9.]+)?' "$f1" | sed 's/Solving with solver //'); grep -q 'defaulting to Boolector' "$f1" && sol="${sol:-Boolector}(padrao)"
  vcc=$(cat "$DIR/${m}"_run*.log | grep -aoE 'Generated [0-9]+ VCC' | awk '{print $2}' | sort -u | tr '\n' '/' | sed 's#/$##')
  ver_=$(grep -aoE 'VERIFICATION (SUCCESSFUL|FAILED)' "$f1" | tail -1 | awk '{print $2}')
  md=$(med "${ts[@]}")
  printf "%-13s %-8s %-24s %-10s %-9s %-5s %-8s %s\n" "$m" "$ver" "${sol:-?}" "$campo" "$md" "$vcc" "${ver_:-?}" "${ts[*]}"
  totg=$(echo "$totg + $md" | bc); [ "$m" != GPS ] && tot=$(echo "$tot + $md" | bc)
done
echo "Total sem GPS: ${tot} s   |   com GPS: ${totg} s"
