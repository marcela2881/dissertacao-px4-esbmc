# Mapa das verificações: fases × ESBMC, CBMC e teste unitário (replay)

Atualizado em 19/10/2026. Legenda: ✅ feito · ⏳ rodando · ❌ falta · — não se aplica.
"Teste unitário" = replay: o código **original** do PX4 executado com g++ usando os valores do
contraexemplo. Só se aplica a testes com contraexemplo (FAILED).

## Fase 1: campanha original (12 harnesses do artigo, `saida/leva1/`)

| Harness | ESBMC 8.5 | CBMC 5.95 | Teste unitário (replay) |
|---|---|---|---|
| Barometer, Battery, EKF2, Expo, MAVLink, Magnetometer, PWM, RC_Update | rápido ✅ · oficial ⏳ | rápido ✅ · oficial ⏳ | — (sem contraexemplo) |
| Navigator | PARSING ERROR no 8.5 ✅ (registrado) | rápido ✅ (SUCCESSFUL após prelúdio) · oficial ⏳ | — |
| VTOL | rápido ✅ · oficial ⏳ | rápido ✅ (FAILED = modelo de `sqrtf` do CBMC) · oficial ⏳ | — |
| **GPS** | FAILED ✅ (rápido e oficial) | Timed out ✅ (rápido e oficial) | ❌ **`replay_gps.cpp` pronto, falta rodar** |
| **IMU BMI088** (temperatura e FIFO) | FAILED ✅ · oficial ⏳ | FAILED ✅ · oficial ⏳ | ❌ **`replay_bmi088.cpp` pronto, falta rodar** |

Depois: repetir `./comparar.sh leva1 oficial` **sozinha, à noite** (os tempos de 19/10 ficaram
contaminados por rodadas simultâneas).

## Fase 2: validação (`saida/fase2/harnesses/`, cópias de `cbmc/` e `seeded_tests/`)

| Harness | Feito na fase 2 original | ESBMC 8.5 | CBMC 5.95 | Teste unitário |
|---|---|---|---|---|
| IMU reduzido (`test_bmi088_cbmc.cpp`) | CBMC: FAILED (temperatura e FIFO) | ❌ | ❌ refazer no protocolo novo | coberto pelo replay do IMU |
| ArduPilot BMI088 | CBMC: FAILED (temperatura) | ❌ | ❌ refazer | — (outro autopiloto) |
| Patch FIFO / patch temperatura | CBMC: 0/2 e 0/4 | ❌ | ❌ refazer | — |
| GPS `len < 200` (sem e com guarda) | CBMC: 0/7 e 0/7 (**não isola a guarda**) | ❌ | ❌ refazer | — |
| GPS semeado `len ≥ 190` sem guarda | CBMC: FAILED com unwind 2, 3, 5 | ❌ | ❌ refazer | coberto pelo replay do GPS |
| **GPS semeado sem × com guarda** (`gps_guarda_semeado.cpp`, **novo**) | nunca feito | ❌ | ❌ | — |
| 10 testes semeados (Boolector × Z3) | tabela do `MAPEAMENTO.md` **não preenchida** | ❌ (Boolector e Z3) | ❌ | — (sintéticos) |
| Reexecuções com outras versões (8.4, 7.10, 8.2) | ✅ (`reexecucoes_2026-10-09/`) | coberto pela fase 1 em 8.5 | — | — |
| Boolector × Z3 em GPS, MAVLink, EKF2 | ✅ ESBMC 8.1 (`validacao_esbmc_8.1/`) | opcional | — | — |

Tudo isso roda com **um comando**: `./rodar_fase2.sh rapido` (depois `oficial`).

## Pacote v5: 8 candidatos de outros módulos (`saida/v5/`, pacote de 12/10/2026)

Não são módulos de saída. Os harnesses já citam saídas do ESBMC 8.1 e 8.4, mas **não há logs
no repositório** e nada foi rodado com o ESBMC 8.5 na máquina oficial. Tudo roda com um comando
(`cd v5 && ./rodar_tudo.sh`), que faz ESBMC, CBMC e o teste unitário (replay com sanitizador) e
compara com o esperado. Conferido aqui: o CBMC 5.95 lê todos os harnesses (só usam headers de C,
não precisam do prelúdio) e todos os replays compilam com g++ 13.3.

| # | Módulo | Situação | ESBMC 8.5 | CBMC 5.95 | Teste unitário (replay) |
|---|---|---|---|---|---|
| 1 | `Common.cpp` (commander) | retirado (premissa errada) | ❌ | ❌ | — (não é defeito) |
| 2 | `externalChecks.cpp` (commander) | defeito, corrigido no PX4 (PR #28598) | ❌ original + correção | ❌ original + correção | ❌ ASan |
| 3 | `ft7_technologies.cpp` (sensor de vento) | defeito, corrigido no PX4 (PR #28600) | ❌ original + correção | ❌ original + correção | ❌ UBSan (2 cenários, antes e depois) |
| 4 | `sumd.cpp` (rádio RC) | **defeito, ainda no PX4** (issue em rascunho) | ❌ | ❌ | ❌ UBSan + efeito no canal 32 |
| 5 | `logger.cpp` | sem violação de memória (truncamento latente) | ❌ | ❌ (só a sondagem do `snprintf`) | — (não é defeito) |
| 6 | `IridiumSBD.cpp` (telemetria satelital) | defeito, corrigido no PX4 (PR #28599) | ❌ original + correção | ❌ original + correção | ❌ ASan |
| 7 | `bst.cpp` (telemetria) | falso positivo (harness antigo sem as guardas) | ❌ antigo + fiel | ❌ fiel | — (não é defeito) |
| 8 | `MicroStrain.cpp` (INS) | **defeito, ainda no PX4** (issue em rascunho) | ❌ | ❌ | ❌ ASan |

Os rascunhos de issue (`v5/issues_px4/`) dependem da aprovação do orientador antes de publicar.

## Fase 3: segunda campanha (módulos de saída, código original, `saida/modulos/`)

| Módulo | ESBMC 8.5 | CBMC 5.95 | Teste unitário (replay) |
|---|---|---|---|
| PCA9685 | oficial ✅ | ❌ (`./comparar.sh pca9685 rapido`; esperado PARSING ERROR) | T3 ✅ (`replay_pca9685.cpp`) · T4 — (inalcançável) |
| RoboClaw | oficial ✅ + correção ✅ | ❌ (esperado PARSING ERROR) | R1 e R3 ✅ (original e corrigido) |
| voxl_esc | oficial ✅ | ❌ **(C: deve dar veredito)** | — (tudo SUCCESSFUL) |
| tap_esc | oficial ✅ (confirmar T1–T4 no log) | ❌ (esperado PARSING ERROR) | — |
| dshot | rápido ✅ (T3 refeito, falta rodar) · oficial ❌ | ❌ (esperado PARSING ERROR) | — até agora |
| uavcan, px4io, mixer_module, control_allocation, control_allocator | ❌ harness a escrever | ❌ | ❌ |

## Ordem sugerida (nunca duas rodadas ao mesmo tempo)

| # | Comando (em `~/verificacoes`, depois de baixar a versão nova) | Tempo |
|---|---|---|
| 1 | esperar a oficial da 1ª leva terminar (`cat logs/2026-10-19_bruno-pc_comparacao_oficial/RESUMO_COMPARACAO_leva1.txt`) | — |
| 2 | `./replay/rodar_replay.sh` (fase 1 GPS e IMU + fase 3 RoboClaw e PCA9685) | ~2 min |
| 3 | `SO_TESTES="3" ./rodar_modulo.sh dshot rapido` | ≤ 5 min |
| 4 | `./rodar_fase2.sh rapido` | ~15–40 min |
| 5 | `./comparar.sh voxl_esc rapido`, `pca9685`, `roboclaw`, `tap_esc`, `dshot` | ~30 min |
| 5b | `cd v5 && nohup ./rodar_tudo.sh > ../v5_saida.txt 2>&1 &` (pacote v5: ESBMC, CBMC e replay; última linha esperada "Tudo como esperado.") | 1–3 h |
| 6 (noite) | `nohup ./comparar.sh leva1 oficial > comp_leva1.txt 2>&1 &` (tempos limpos) | até 4 h |
| 7 (noite) | `nohup ./rodar_fase2.sh oficial > fase2_oficial.txt 2>&1 &` | horas |
| 8 (noite) | `nohup ./comparar.sh todos oficial > comp_todos.txt 2>&1 &` | horas |
| — | harnesses dos 5 módulos que faltam (eu escrevo; você roda rápido → oficial → comparar) | dias |
