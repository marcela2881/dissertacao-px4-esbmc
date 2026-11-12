# TUDO_FINAL — versão 4 (08/10/2026) — ver CORRECOES_v4.md

> v4 corrige o v3 após a revisão crítica (review_report.md). O v3 não foi sobrescrito.

Artefato de reprodução dos candidatos a vulnerabilidade verificados no PX4
Autopilot v1.16 com ESBMC, CBMC e sanitizadores.

Esta versão incorpora o parecer do orientador sobre a reprodução independente do
pacote, feita em macOS/aarch64 com ESBMC 8.4.0, Bitwuzla 0.9.1 e CBMC 6.8.0.

## Como o pacote está organizado

Tudo que existia na versão anterior está preservado, e o que foi refeito em
agosto de 2026 fica em pastas separadas. Em nenhum lugar um arquivo antigo foi
sobrescrito.

```
guia_reproducao.md          instruções de reprodução (v4)
CORRECOES_v4.md             o que mudou e o que falta rodar

1_harnesses/
    ANTIGO/                 harnesses das rodadas originais
    ATUAL/                  harness reescrito em ago/2026

2_replays/
    ANTIGO/                 replays das rodadas originais
    ATUAL/                  replays reescritos em ago/2026

3_resultados_ANTIGO/        logs das rodadas originais, intocados
4_resultados_ATUAL/         logs da re-execução de ago/2026

5_submissao_px4/            rascunhos de submissão à comunidade
6_documentos/               relatório ao orientador e rascunho das issues
```

Os arquivos em `ANTIGO/` permanecem para rastreabilidade, mas **não devem ser
usados para reproduzir os achados** nos casos indicados abaixo. Os motivos estão
detalhados em `4_resultados_ATUAL/LEIA-ME.md`.

## O que foi substituído e por quê

| Antigo | Atual | Motivo |
|---|---|---|
| `test_microstrain_port_v2.cpp` | `test_microstrain_port_v3.cpp` | o v2 fixava `max_len = PORT_SIZE` por constante, sem verificar se o estado crítico é alcançável a partir da entrada |
| `replay_bst_real.cpp` | `replay_bst_probe_only.cpp` | o antigo abortava no `main()` antes de chamar `bst_probe_real()`; o `probe()` verbatim nunca executava |
| `replay_microstrain.cpp` | `replay_microstrain_v3.cpp` | mesmo problema do harness: `max_len` atribuído em vez de calculado |

Nenhum harness ou replay dos candidatos 1, 2, 3, 5 e 6 foi alterado — nesses
casos só os comandos mudaram.

## Correções no guia de reprodução

| Candidato | O que mudou |
|---|---|
| 4 — `sumd.cpp` | ESBMC passa de Timeout a FAILED com `--unwind 70`; ASan não detecta (intra-objeto), UBSan confirma |
| 5 — `logger.cpp` | comando exige `--max-k-step 320`; log incluído no pacote pela primeira vez |
| 6 — `IridiumSBD.cpp` | CBMC passa de UNKNOWN a FAILED; exige `-D'ssize_t=long'` |
| 7 — `bst.cpp` | replay substituído pelo que alcança `probe()` |
| 8 — `MicroStrain.cpp` | harness reescrito com entrada não-determinística |

## Situação dos oito candidatos

| # | Módulo | Classe de entrada | Situação |
|---|---|---|---|
| 1 | `Common.cpp` | barramento interno de eventos | **retirado** — harness assumia estado inalcançável (idx>SIZE); com idx<=SIZE, 0 de 10 falhas |
| 2 | `externalChecks.cpp` | uORB, componente externo | achado — três ferramentas |
| 3 | `ft7_technologies.cpp` | sensor serial (UART) | achado — três ferramentas |
| 4 | `sumd.cpp` | rádio (protocolo RC) | achado — severidade prática baixa, escrita intra-objeto |
| 5 | `logger.cpp` | saída (gravação em disco) | sem violação para os comprimentos modelados (k=300 vale p/ o harness) |
| 6 | `IridiumSBD.cpp` | telemetria satelital (transmissão) | achado — três ferramentas |
| 7 | `bst.cpp` | telemetria I2C | achado — `probe()` verbatim executa |
| 8 | `MicroStrain.cpp` | INS, string de porta serial | achado — após reescrita do harness |

Seis achados alcançáveis no modelo (2, 3, 4, 6, 7, 8), um retirado (1) e um sem violação no modelo (5).

Nenhum é defeito confirmado em produção: a alcançabilidade no alvo físico não
foi estabelecida em nenhum dos casos, e isso está registrado em cada issue.

## Reprodução

Ver `guia_reproducao.md`. Todos os comandos foram conferidos na re-execução de
19/08/2026, e os que exigiam ajuste estão corrigidos.
