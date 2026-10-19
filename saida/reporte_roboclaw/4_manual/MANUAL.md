# Especificação: manual da RoboClaw

A propriedade verificada no R1 ("o byte de dados dos comandos de acionamento fica entre 0 e 127") vem
do manual do fabricante. Os PDFs **não** ficam no repositório (direitos autorais); abaixo estão as
referências e o trecho citado.

## Referência principal (versão atual)

Basicmicro (Ion Motion Control), *RoboClaw Series User Manual*, **Revision 6.0**, firmware 4.3.0 e mais
novo. Fonte oficial: https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf

Seção **"Commands 0 - 7 Compatibility Commands"** (modo Packet Serial):

| Comando | Nome | Página | Texto |
|---|---|---|---|
| 0 | Drive Forward M1 | 48 | "Drive motor 1 forward. Valid data range is 0 - 127. A value of 127 = full speed forward, 64 = about half speed forward and 0 = full stop." |
| 1 | Drive Backwards M1 | 48 | "Drive motor 1 backwards. Valid data range is 0 - 127. A value of 127 full speed backwards, 64 = about half speed backward and 0 = full stop." |
| 4 | Drive Forward M2 | 49 | "Drive motor 2 forward. Valid data range is 0 - 127. A value of 127 full speed forward, 64 = about half speed forward and 0 = full stop." |
| 5 | Drive Backwards M2 | 49 | conferir de passagem (nas Rev. 5 e 5.5: "Drive motor 2 backwards. Valid data range is 0 - 127. …") |

Formato do pacote (mesma página): `Send: [Address, Comando, Value, CRC(2 bytes)]`, `Receive: [0xFF]`.

## Revisões anteriores (mesmo texto)

| Revisão | Firmware | Páginas (cmd 0/1 · cmd 4/5) | Conferido em |
|---|---|---|---|
| 6.0 (atual) | 4.3.0+ | 48 · 49 | 19/10/2026 (texto colado do PDF oficial) |
| 5.5 (PDF de 2016) | 4.1.16+ | 48 · 49 | 19/10/2026 (PDF) |
| 5 (PDF de 2015) | 4.1.11+ | 40 · 41 | 19/10/2026 (PDF) |

## Ligação com o código do PX4

| Código (commit 8be2590) | Comando do manual |
|---|---|
| `Command::DriveForwardMotor1 = 0` (Roboclaw.hpp) | 0 – Drive Forward M1 |
| `Command::DriveBackwardsMotor1 = 1` | 1 – Drive Backwards M1 |
| `Command::DriveForwardMotor2 = 4` | 4 – Drive Forward M2 |
| `Command::DriveBackwardsMotor2 = 5` | 5 – Drive Backwards M2 |

`setMotorSpeed()` escolhe um desses quatro comandos e `sendUnsigned7Bit()` monta o byte `Value`.
