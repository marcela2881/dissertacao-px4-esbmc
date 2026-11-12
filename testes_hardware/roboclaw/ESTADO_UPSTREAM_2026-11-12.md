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

Conferido nos arquivos de cada ramo em 2026-11-12:

| Ramo | Bug 1 (`lroundf(fabs(data) * INT8_MAX)`, failsafe até 257) | Bug 2 (`size_t bytes_written = write`) | Membros não inicializados e `update()` antes de `initializeUART()` |
|---|---|---|---|
| `release/1.18` (`8303efda43cb`) | sim, as duas coisas | sim | sim |
| `release/1.17` e `v1.17.0` | failsafe até 257; `sendUnsigned7Bit` tem outro código (limita `data >= 1.0`), não analisado | sim | sim |
| `release/1.16` | failsafe até 257; `sendUnsigned7Bit` não conferido | sim | sim |

`Roboclaw.cpp`, `Roboclaw.hpp` e `module.yaml` de `release/1.18`
(`8303efda43cb`) são idênticos, byte a byte, aos do commit analisado
(`8be2590c15`): `diff` sem saída nos três arquivos, conferido em 2026-11-12.
A análise formal e o ensaio em hardware valem, portanto, para o código do
driver que está hoje nesse ramo. O restante do firmware (mixer, NuttX) não foi
comparado.

## Decisão (2026-11-12)

A autora decidiu reportar, com autorização do orientador.
`ISSUE_FINAL_com_hardware.md` foi ajustado para declarar a versão afetada
(`release/1.18` e o commit `8be2590c15`) e dizer que o `main` foi refatorado
pelo #29473. A postagem é feita pela autora.

## batmon

`src/drivers/smart_battery/batmon` não tem diferença entre o commit conferido
(`c9b9e4e25e`) e o `main` de 2026-11-11. A issue do batmon continua válida
quanto a esse ponto.
