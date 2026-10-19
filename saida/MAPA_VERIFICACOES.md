# Mapa das verificações: fases × ESBMC, CBMC e teste unitário (replay)

Atualizado em 19/10/2026. Legenda: ✅ feito · ⏳ rodando · ❌ falta · — não se aplica.
"Teste unitário" = replay: o código **original** do PX4 executado com g++ usando os valores do
contraexemplo. Só se aplica a testes com contraexemplo (FAILED).

## Fase 1: campanha original (12 harnesses do artigo, `saida/leva1/`)

| Harness | ESBMC 8.5 | CBMC 5.95 | Teste unitário (replay) |
|---|---|---|---|
| Barometer, Battery, EKF2, Expo, MAVLink, Magnetometer, PWM, RC_Update | rápido ✅ · oficial ✅ | rápido ✅ · oficial ✅ | — (sem contraexemplo) |
| Navigator | PARSING ERROR no 8.5 ✅ (registrado) | rápido ✅ · oficial ✅ (SUCCESSFUL com o prelúdio) | — |
| VTOL | rápido ✅ · oficial ✅ | rápido ✅ · oficial ✅ (FAILED = modelo de `sqrtf` do CBMC) | — |
| **GPS** | FAILED ✅ (rápido e oficial) | Timed out ✅ (rápido e oficial) | ✅ 19/10 (máquina oficial, 16:33): alcance, 2 milhões de chamadas a partir de len 0, máximo 78 (capacidade 79); estado do contraexemplo (len 207) → ASan acusa escrita fora do buffer |
| **IMU BMI088** (temperatura e FIFO) | FAILED ✅ · oficial ✅ | FAILED ✅ · oficial ✅ | ✅ 19/10: temperatura 86,25 °C reproduzida; contador 9088 barrado pela guarda; **novo: leitura fora de `FIFOTransferBuffer` com bytes aleatórios** (ver abaixo) |

**Achado novo do replay (19/10, regra 7 pendente):** `FIFORead` lê até 6 bytes além do fim de
`FIFOTransferBuffer` quando um cabeçalho de quadro de dados aparece nos últimos 6 bytes da janela
lida. Com dados válidos isso exige 32 amostras (buffer cheio) e um quadro de 2 ou 4 bytes antes dos
quadros de dados (modo `fifo_quadro2`: ASan acusa; `fifo_alinhado`: sem erro). Só leitura (2 a 6
bytes da pilha vão para o eixo z da última amostra). O código é igual no `main` de 19/10/2026.
Confrontado com o datasheet (Seções 4.9.3.2 a 4.9.3.6, pp. 20–21): classificado como inalcançável em produção (só com bytes corrompidos na SPI); `fifo_quadro2` e `fifo_alinhado` confirmados na máquina oficial em 19/10. Falta o orientador confirmar.

Depois: repetir `./comparar.sh leva1 oficial` **sozinha, à noite** (os tempos de 19/10 ficaram
contaminados por rodadas simultâneas).

**Oficial da 1ª leva ESBMC × CBMC fechada (19/10/2026, bruno-pc, 3 execuções, mediana, 600 s).**
Os tempos foram medidos com outras rodadas em paralelo; repetir à noite, sozinha, para tempos limpos.

| Módulo | ESBMC 8.5 UA / NUA | CBMC 5.95 UA / NUA |
|---|---|---|
| Barometer | SUCCESSFUL 0,819 / 0,763 s | PARSING ERROR |
| Battery_Monitor | SUCCESSFUL 16,010 / 13,502 s | Timed out |
| EKF2_Covariance | SUCCESSFUL 34,779 / 40,462 s | Timed out |
| Expo_RC | SUCCESSFUL 4,702 / 4,777 s | SUCCESSFUL 1,846 / 1,837 s |
| GPS | FAILED 136,625 / 156,242 s | Timed out |
| IMU_BMI088 | FAILED 0,287 / 0,247 s | FAILED 1,451 / 1,196 s |
| MAVLink | SUCCESSFUL 88,747 / 87,482 s | SUCCESSFUL 26,151 / 24,550 s |
| Magnetometer | SUCCESSFUL 9,894 / 9,774 s | SUCCESSFUL 2,358 / 2,350 s |
| Navigator_Mission | PARSING ERROR | SUCCESSFUL 0,694 / 0,647 s (com o prelúdio) |
| PWM_Output | SUCCESSFUL 0,299 / 0,298 s | SUCCESSFUL 0,612 / 0,572 s |
| RC_Update | SUCCESSFUL 6,122 / 6,316 s | SUCCESSFUL 24,744 / 24,945 s |
| VTOL | SUCCESSFUL 7,128 / 7,048 s | FAILED 1,033 / 1,092 s (modelo de `sqrtf` do CBMC) |

Os vereditos concordam em 6 de 12 (Expo, IMU, MAVLink, Magnetometer, PWM, RC);
o CBMC não chega a veredito em Battery, EKF2 e GPS (Timed out) e não lê o Barometer; o ESBMC 8.5
não lê o Navigator; no VTOL o CBMC dá FAILED pelo modelo de `sqrtf`.

## Fase 2: validação (`saida/fase2/harnesses/`, cópias de `cbmc/` e `seeded_tests/`)

Rápida feita em 19/10 (bruno-pc, ESBMC 8.5.0 e CBMC 5.95.1, 300 s, sem asserções de desdobramento).
Falta a oficial (`./rodar_fase2.sh oficial`, à noite).

| Harness | ESBMC 8.5 (Boolector) | ESBMC 8.5 (Z3) | CBMC 5.95 | Teste unitário |
|---|---|---|---|---|
| IMU reduzido | FAILED ✅ | — | FAILED ✅ (temperatura e FIFO) | replay do IMU ✅ |
| ArduPilot BMI088 | FAILED ✅ | — | FAILED ✅ | — (outro autopiloto) |
| Patch FIFO / patch temperatura | SUCCESSFUL ✅ | — | SUCCESSFUL ✅ | — |
| GPS `len < 200` sem e com guarda | SUCCESSFUL ✅ | — | SUCCESSFUL ✅ (237 s e 254 s) | — |
| GPS semeado `len ≥ 190` (antigo) | FAILED ✅ | — | FAILED ✅ | replay do GPS ✅ |
| **GPS semeado sem guarda** (novo) | **FAILED ✅** | — | **FAILED ✅** | — |
| **GPS semeado com guarda** (novo) | **SUCCESSFUL ✅** | — | **SUCCESSFUL ✅** | — |
| 8 semeados sem laço | FAILED ✅ | FAILED ✅ | FAILED ✅ | — (sintéticos) |
| 2 semeados com laço | FAILED ✅ | FAILED ✅ | **Timed out** | — |

O caso do patch do GPS está fechado: com a mesma pré-condição (`len ≥ 190`), sem a guarda as duas
ferramentas acham a violação e com a guarda as duas dão SUCCESSFUL.

## Pacote v5: 8 candidatos de outros módulos (`saida/v5/`, pacote de 12/10/2026)

Não são módulos de saída. Os harnesses já citam saídas do ESBMC 8.1 e 8.4, mas **não há logs
no repositório** e nada foi rodado com o ESBMC 8.5 na máquina oficial. Tudo roda com um comando
(`cd v5 && ./rodar_tudo.sh`), que faz ESBMC, CBMC e o teste unitário (replay com sanitizador) e
compara com o esperado. Conferido aqui: o CBMC 5.95 lê todos os harnesses (só usam headers de C,
não precisam do prelúdio) e todos os replays compilam com g++ 13.3.

| # | Módulo | Situação | ESBMC 8.5 | CBMC 5.95 | Teste unitário (replay) |
|---|---|---|---|---|---|
| 1 | `Common.cpp` (commander) | retirado (premissa errada) | ✅ | ✅ | — (não é defeito) |
| 2 | `externalChecks.cpp` (commander) | defeito, corrigido no PX4 (PR #28598) | ✅ original + correção | ✅ original + correção | ✅ ASan |
| 3 | `ft7_technologies.cpp` (sensor de vento) | defeito, corrigido no PX4 (PR #28600) | ✅ original + correção | ✅ original + correção | ✅ UBSan (2 cenários, antes e depois) |
| 4 | `sumd.cpp` (rádio RC) | **defeito, ainda no PX4** (issue em rascunho) | ✅ | ✅ | ✅ UBSan + efeito no canal 32 |
| 5 | `logger.cpp` | sem violação de memória (truncamento latente) | ✅ | ✅ (só a sondagem do `snprintf`) | — (não é defeito) |
| 6 | `IridiumSBD.cpp` (telemetria satelital) | defeito, corrigido no PX4 (PR #28599) | ✅ original + correção | ✅ original + correção | ✅ ASan |
| 7 | `bst.cpp` (telemetria) | falso positivo (harness antigo sem as guardas) | ✅ antigo + fiel | ✅ fiel | — (não é defeito) |
| 8 | `MicroStrain.cpp` (INS) | **defeito, ainda no PX4** (issue em rascunho) | ✅ | ✅ | ✅ ASan |

**1ª execução (19/10/2026, bruno-pc): 47 de 48 resultados conferem com o esperado.** Atenção: o
`rodar_tudo.sh` usou o `esbmc` do PATH, que é o **ESBMC 8.2.0**, e não o 8.5 (replays com clang 18.1.3;
PX4 `main` conferido em `1c8874d41a`, de 19/10/2026). Única divergência: `4_sumd/esbmc.txt` deu OUTRO
(provável tempo-limite de 1 h; o CBMC deu FAILED, 2 de 38 propriedades, e o replay com UBSan reproduziu).
**2ª execução (19/10/2026, `resultados_20261019_1423`), ESBMC 8.5 + Boolector (`esbmc_boolector.sh`): "Tudo como esperado", todos os resultados conferem, inclusive o sumd.** (O sumd do 8.2 era tempo-limite de 1 h com o Z3, solver padrão.)

Os rascunhos de issue (`v5/issues_px4/`) dependem da aprovação do orientador antes de publicar.

## Fase 3: segunda campanha (módulos de saída, código original, `saida/modulos/`)

| Módulo | ESBMC 8.5 | CBMC 5.95 | Teste unitário (replay) |
|---|---|---|---|
| PCA9685 | oficial ✅ | ❌ (`./comparar.sh pca9685 rapido`; esperado PARSING ERROR) | T3 ✅ (`replay_pca9685.cpp`) · T4 — (inalcançável) |
| RoboClaw | oficial ✅ + correção ✅ | ❌ (esperado PARSING ERROR) | R1 e R3 ✅ (original e corrigido) |
| voxl_esc | oficial ✅ | ❌ **(C: deve dar veredito)** | — (tudo SUCCESSFUL) |
| tap_esc | oficial ✅ (confirmar T1–T4 no log) | ❌ (esperado PARSING ERROR) | — |
| dshot | rápido ✅ (T3 refeito, falta rodar) · oficial ❌ | ❌ (esperado PARSING ERROR) | — até agora |
| mixer_module | ❌ harness pronto (19/10, 1 canal; T2 e T4 ~150 s na conferência), falta rodar | ❌ (esperado PARSING ERROR) | só se der defeito |
| control_allocation | ❌ harness pronto (19/10, `modulos/09_control_allocation`); conferência: T1 e T2 SUCCESSFUL (5 s e 10 s), T0 e T3 passam de 90 s, falta rodar | ❌ | só se der defeito |
| uavcan, px4io, control_allocator | ❌ harness a escrever | ❌ | ❌ |

## Ordem sugerida (nunca duas rodadas ao mesmo tempo)

| # | Comando (em `~/verificacoes`, depois de baixar a versão nova) | Tempo |
|---|---|---|
| 1 | esperar a oficial da 1ª leva terminar (`cat logs/2026-10-19_bruno-pc_comparacao_oficial/RESUMO_COMPARACAO_leva1.txt`) | — |
| 2 | ✅ `./replay/rodar_replay.sh` (repetir com a versão nova: GPS "alcance" e BMI088 `fifo_quadro2`) | ~2 min |
| 3 | `SO_TESTES="3" ./rodar_modulo.sh dshot rapido` | ≤ 5 min |
| 4 | ✅ `./rodar_fase2.sh rapido` | ~15–40 min |
| 5 | `./comparar.sh voxl_esc rapido`, `pca9685`, `roboclaw`, `tap_esc`, `dshot` | ~30 min |
| 5b | `cd v5 && nohup ./rodar_tudo.sh > ../v5_saida.txt 2>&1 &` (pacote v5: ESBMC, CBMC e replay; última linha esperada "Tudo como esperado.") | 1–3 h |
| 6 (noite) | `nohup ./comparar.sh leva1 oficial > comp_leva1.txt 2>&1 &` (tempos limpos) | até 4 h |
| 7 (noite) | `nohup ./rodar_fase2.sh oficial > fase2_oficial.txt 2>&1 &` | horas |
| 8 (noite) | `nohup ./comparar.sh todos oficial > comp_todos.txt 2>&1 &` | horas |
| — | harnesses dos 5 módulos que faltam (eu escrevo; você roda rápido → oficial → comparar) | dias |
