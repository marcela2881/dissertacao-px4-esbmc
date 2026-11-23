# Auditoria de números (23/11/2026)

Método: cada número do pôster v18, dos slides v3 e dos trechos novos do capítulo de resultados foi comparado com o log ou a planilha que o origina. Os tempos da primeira campanha foram recalculados com `tabela_ii_dos_logs.sh` sobre `medianas_logs/` (36 logs, ESBMC 8.1.0, Boolector 3.2.4).

## Tempos da primeira campanha (pôster, slides, Tabela II)
| Módulo | Mediana recalculada (s) | Valor usado | Confere |
|---|---|---|---|
| GPS uBlox | 64,857 | 64,9 | sim |
| MAVLink | 49,176 | 49,2 | sim |
| EKF2 Covariance | 12,213 | 12,2 | sim |
| RC Update | 9,468 | 9,5 | sim |
| Battery Monitor | 6,738 | 6,7 | sim |
| Magnetometer | 6,630 | 6,6 | sim |
| Expo RC | 4,870 | 4,9 | sim |
| VTOL | 3,966 | 4,0 | sim |
| Barometer | 0,368 | 0,4 | sim |
| PWM Output | 0,161 | 0,2 | sim |
| IMU BMI088 | 0,122 | 0,1 | sim |
| Navigator/Mission | 0,100 | 0,1 | sim |
| Total sem GPS | 93,812 | ≈94 | sim |
| Total com GPS | 158,669 | ≈159 | sim |

Pôster: "7 other harnesses ≤ 6,7 s" = Battery, Magnetometer, Expo, VTOL, Barometer, PWM e Navigator (maior valor 6,7); correto. "≤ 65 s" por harness: o maior é 64,857; correto. "Under 90 s": o maior é 64,9; correto.

## Tamanhos (Tabela I)
Soma dos harnesses: 294+319+82+129+242+208+253+291+296+239+278+146 = 2.777 (≈2.800 no texto). Soma dos alvos: 1712+700+3624+303+449+337+769+497+385+792+246+591 = 10.405 (≈10.400 no texto). PX4: 710.259 linhas em `src/` (≈710k).

## Campanha suplementar de 23/11 (capítulo de resultados)
| Afirmação no texto | Fonte | Confere |
|---|---|---|
| GPS k=211 original: encerrado aos 1.225 s (saída 15) | `evidencia_execucoes_2026-11-23/pendencias_A_B_C/C_gps_k211_original.log` | sim |
| GPS k=211 propriedade limpa: abortou aos 1.293 s (saída 134, `corrupted double-linked list`) | `.../C_gps_k211_prop_limpa.log` | sim |
| Monitor de memória encerrou o GPS às 06:57 (1.119 MB) e a memória ficou abaixo de 1 GiB às 07:16–07:18 | `evidencia_execucoes_2026-11-23/guarda_memoria.log` | sim |
| Battery 16 células: 3 execuções sem veredito em 120 s; 278 s sem violação com 900 s | `pendencias_A_B_C/RESUMO.txt`; `inconclusivos_900s/RESUMO.txt` | sim |
| Testes separados (11 módulos) | `pendencias_A_B_C/RESUMO.txt` | sim |
| Inconclusivos 900 s: EKF2 t1, GPS t0/t1/t3 sem veredito; GPS t2 289 s, VTOL t0 154 s, VTOL t1 496 s sem violação | `inconclusivos_900s/RESUMO.txt` | sim |
| CBMC nos dez: oito sem violação (0–110 s), Battery e EKF2 sem veredito em 900 s | `cbmc_dez/RESUMO.txt` | sim |
| CBMC 7.200 s: Battery e EKF2 sem veredito nas quatro execuções | `cbmc_longo_7200s/RESUMO.txt` | sim |
| Máquina: AMD Ryzen 7 5700U, 16 núcleos lógicos, 11 GiB, Ubuntu 24.04.2, ESBMC 8.2.0, Z3, CBMC 5.95.1 | `ambiente.txt` de cada pasta | sim |

## Ressalvas
- O Battery com 16 células tem dois tempos no texto (190 s na metodologia, 278 s nos resultados). Os dois vêm de execuções diferentes; a segunda rodou com outro experimento em paralelo na mesma máquina. Só o veredito é comparável; isto está dito no capítulo de metodologia.
- O log individual do Battery sem asserções no CBMC de 7.200 s foi regravado por uma segunda execução do script; o resultado dele está no `RESUMO.txt`, não no log.
- Os dois logs da parte C estão rotulados como "(timeout?)" no `RESUMO.txt`, mas não foram timeout.
- O estado do PR #27576 foi conferido na página do PR (capturas de 23/11/2026); a data exata do fechamento ("3 semanas atrás") não foi lida.
