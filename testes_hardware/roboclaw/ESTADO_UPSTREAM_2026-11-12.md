# RoboClaw: estado do driver no PX4 `main` em 2026-11-12

Conferido em 2026-11-12 contra `PX4/PX4-Autopilot` `main`
(`8e94784e1dc0c4bbaa670da883c8024d9dc6ead2`, de 2026-11-11).

## O driver foi reescrito no `main`

Commit `cf241e9e14a899cf4385aff32b8eb21e8765c3b1`, de 2026-10-28:
"refactor(Roboclaw): Refactor driver with output module interface fixes and
multiple support (#29473)". O commit analisado na dissertação (`8be2590c15`,
de 2026-06-04) é anterior a ele.

| Defeito (commit `8be2590c15`) | Situação no `main` |
|---|---|
| Bug 1: byte de dados > 127 com *failsafe* até 257 | não ocorre mais: `module.yaml` limita `failsafe` a 254 e `setMotorSpeed` aplica `math::constrain(value, -1.f, 1.f)` antes de multiplicar por `INT8_MAX` |
| Bug 2: retorno de `write()` em `size_t` | não ocorre mais: `ssize_t bytes_written = _uart.writeBlocking(...)`, comparado com `static_cast<ssize_t>(packet_size)` |
| Terceiro defeito: `select()` com `fd_set`/`timeval` não inicializados antes de `initializeUART()` | não ocorre mais: os dois membros não existem (o driver usa `device::Serial`) e `Run()` chama `init()` antes de `_mixing_output.update()` |

## Onde o código antigo ainda está

Os ramos `release/1.16`, `release/1.17` e `release/1.18` ainda trazem as linhas
`lroundf(fabs(data) * INT8_MAX)` e `size_t bytes_written = write(...)`
(conferido por busca textual no `Roboclaw.cpp` de cada ramo; não foi conferido
o `module.yaml` nem o `.hpp` desses ramos).

## Consequência

`ISSUE_FINAL_com_hardware.md` e `PR.md` descrevem o defeito como presente no
código atual. Isso deixou de ser verdade para o `main`. **Não postar como
está.** As alternativas (reportar contra os ramos de release, ou não reportar e
registrar na dissertação como defeito confirmado e corrigido de forma
independente) dependem de decisão da autora com o orientador.

## batmon

`src/drivers/smart_battery/batmon` não tem diferença entre o commit conferido
(`c9b9e4e25e`) e o `main` de 2026-11-11. A issue do batmon continua válida
quanto a esse ponto.
