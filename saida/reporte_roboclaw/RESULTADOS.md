# Resultados: driver RoboClaw do PX4 (R1 e R3)

Código verificado: PX4-Autopilot, commit `8be2590c1529517ccd54fbc09ae138402517baf4`,
`src/drivers/roboclaw/Roboclaw.cpp`. O mesmo código está na `main` (conferido em 19/10/2026).

Máquina: PC "bruno-pc" (Windows + WSL Ubuntu 24.04), ESBMC 8.5.0 com Boolector, g++ 13.3.0.

## Especificação

Basicmicro, *RoboClaw Series User Manual*, Revision 6.0 (firmware 4.3.0+), seção
"Commands 0 - 7 Compatibility Commands", pp. 48–49, comandos 0, 1, 4 e 5:
*"Valid data range is 0 - 127."* O mesmo texto aparece na Rev. 5.5 (pp. 48–49) e na Rev. 5 (pp. 40–41).

POSIX: `write()` devolve −1 em caso de erro.

## 1. Verificação formal (ESBMC), código ORIGINAL

| Teste | Propriedade | Rodada oficial (18/10/2026, 3 execuções) | Reporte (19/10/2026) |
|---|---|---|---|
| R1 (T1) | byte de dados ≤ 127 para saídas em [0, 257] | FAILED 3/3 | FAILED (5,004 s) |
| R2 (T2) | byte de dados ≤ 127 para saídas em [0, 254] | SUCCESSFUL 3/3 | SUCCESSFUL (6,724 s) |
| R3 (T3) | `select()` não é chamado depois de `write()` = −1 | FAILED 3/3 | FAILED (45,424 s) |

Opções: `--function harness_main --unwind 20 --overflow-check --ub-shift-check --default-solver boolector`.

## 2. Teste unitário (replay): `Roboclaw.cpp` ORIGINAL executado com g++ e UART simulada

| saída (outputs[0]) | comando | byte | situação |
|---|---|---|---|
| 0 | DriveBackwardsMotor1 | 127 | ok |
| 127 | DriveBackwardsMotor1 | 0 | ok |
| 200 | DriveForwardMotor1 | 73 | ok |
| 254 | DriveForwardMotor1 | 127 | ok |
| 254,5 | DriveForwardMotor1 | **128** | VIOLA |
| 255 | DriveForwardMotor1 | **128** | VIOLA |
| 256 | DriveForwardMotor1 | **129** | VIOLA |
| 257 | DriveForwardMotor1 | **130** | VIOLA |

R3: com `write()` sempre devolvendo −1, `select()` foi chamado **7 vezes**. `readEncoder()` termina
em ERROR só porque a resposta simulada falha no CRC, não porque a falha de escrita foi detectada.

Resumo: 5 situações reproduzidas.

Reproduções mínimas (sem PX4):
- `minimo_r1`: 254 → 127, 255 → 128, 256 → 129, 257 → 130
- `minimo_r3`: `bytes_written = 18446744073709551615`, e `writeCommand()` devolve OK

### Reprodução no formato da issue (`repro_roboclaw.cpp`): preparação, execução, assert violado

Rodada na máquina oficial (bruno-pc, g++ 13.3), 19/10/2026:

```
R1: outputs[0] = 257 (RBCLW_FAIL1 = 257, flight termination)
  sent: command 0, data byte 130
Assertion `p[2] <= 127 && "drive command data byte must be 0..127"' failed.
R3: write() returns -1 (errno = EIO)
Assertion `!g_last_write_failed && "select() called after write() returned -1"' failed.
```

Com a correção, os dois testes terminam sem violar os asserts (byte 127; `select()` não é chamado).

### Proteções conferidas no código (comentário 3 do orientador)

- R1: nenhuma função entre `actualFailsafeValue()` e `write()` limita o valor; o mixer só limita no
  caminho normal (`output_limit_calc`); 257 está dentro do `module.yaml`; a terminação é alcançável armado
  por `MAV_CMD_DO_FLIGHTTERMINATION` ou por ação de failsafe "Terminate" (`NAV_RCL_ACT`, `NAV_DLL_ACT`);
  `CBRK_FLIGHTTERM` só desliga a terminação do detector de falhas.
- R3: nada trata o erro entre `write()` e a comparação; a falha só aparece depois, no timeout de 11 ms do
  `select()`, com a mensagem errada ("ACK timeout"/"Select timeout"). Por isso o título diz "not detected",
  e não "treated as success".

## 3. Com a CORREÇÃO (`5_pr/correcao_roboclaw.patch`), 19/10/2026

| Verificação | Original | Corrigido |
|---|---|---|
| ESBMC R1, opções da oficial | FAILED | **SUCCESSFUL** (5,650 s) |
| ESBMC R3, só a propriedade do R3* | FAILED (15,466 s), `assertion !g_write_failed` no `select` | **SUCCESSFUL** (28,857 s) |
| Replay R1 | bytes 128–130 | todos **127** |
| Replay R3 | `select()` 7× | `select()` **0×** |
| Situações reproduzidas | 5 | **0** |

\* Sem `--overflow-check` e `--ub-shift-check`, com as mesmas opções no original e no corrigido. Com
essas opções, o corrigido para em `arithmetic overflow on shl` em `swapBytesInt32()`
(`buffer[0] << 24`). Esse é um falso positivo já conhecido (R0): o ESBMC aplica a regra do C a um
código C++17, em que esse deslocamento é definido. Não tem relação com R1 nem com R3.

## Não testado

- Hardware: o que a RoboClaw faz ao receber bytes 128–130.
- CBMC: não lê o C++ do PX4 (`PARSING ERROR`, por causa dos literais `11_ms`).
