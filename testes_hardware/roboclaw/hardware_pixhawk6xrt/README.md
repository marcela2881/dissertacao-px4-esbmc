# Validação em hardware do Bug 1 do driver RoboClaw — Pixhawk 6X RT + ESP32

Data do ensaio: 2026-11-12. Este registro substitui a conclusão da primeira
tentativa (Arduino Uno, ver `../NOTA_VALIDACAO_HARDWARE.md`), que não tinha
conseguido ler o protocolo.

## Resultado

Com `RBCLW_FAIL1 = 257` e a terminação de voo ativa, o Pixhawk 6X RT transmitiu
na TELEM2, de forma contínua, o pacote

    80 00 82 8A 90

isto é: endereço `0x80`, comando `0` (drive M1), **byte de dados `0x82` = 130**,
CRC16 `0x8A90`. O manual do RoboClaw só admite 0–127 nesse byte. É o valor que a
verificação formal e a UART emulada previam (257 → 130).

## O que é real e o que é emulado

| Elemento | Situação |
|---|---|
| Controladora | **Real**: Pixhawk 6X RT (`px4_fmu-v6xrt_default`) |
| Firmware | **Real**: PX4 no commit `8be2590c15`, com **uma alteração de 2 linhas** (abaixo) |
| Caminho exercitado | **Real**: Commander → `actuator_armed.termination` → MixingOutput → `Roboclaw::updateOutputs` → `sendUnsigned7Bit` → UART |
| Fio | **Real**: TX/RX da TELEM2 (`/dev/ttyS5`), 38400 8N1, 3,3 V |
| RoboClaw | **Emulado** por um ESP32 (`roboclaw_emulador_esp32/`), que responde às consultas e devolve ACK |
| Motor | **Ausente**. O efeito físico do byte 130 num RoboClaw real **não foi testado** |

Por que emular: o driver consulta o status ao iniciar (comando 90) e se encerra
se não receber resposta; só escutando a linha, ele nunca envia comando de motor.

## Alteração no firmware (declarada)

Em `src/drivers/roboclaw/Roboclaw.hpp`:

```diff
-       fd_set _uart_fd_set;
-       struct timeval _uart_fd_timeout;
+       fd_set _uart_fd_set{};
+       struct timeval _uart_fd_timeout{};
```

Sem ela, no firmware original, o driver travava a fila `wq:hp_default` nesta
placa antes de abrir a UART: `Roboclaw::Run` chama `_mixing_output.update()`
antes de `initializeUART()`, o mixer força um primeiro `updateOutputs`, e o
driver chama `select()` com `fd_set`/`timeval` não inicializados. A alteração
não toca no caminho do Bug 1 (`updateOutputs`/`sendUnsigned7Bit`).
Esse travamento é um terceiro defeito, observado só neste ensaio; ainda não foi
verificado formalmente nem reportado.

## Configuração

`RBCLW_SER_CFG=102` (TELEM2), `SER_TEL2_BAUD=38400`, `RBCLW_ADDRESS=128`,
`RBCLW_FUNC1=101`, `RBCLW_FAIL1=257`. Bancada, sem motor e sem hélices.
Sequência no console: `commander arm -f`, `commander termination`.
Estado confirmado na placa: `navigation mode: Termination`,
`actuator_armed: armed: True, termination: True`.

## Evidência: `roboclaw_esp32_captura_trecho.txt`

Trecho do Monitor Serial do ESP32 (22:29:17 a 22:31:35), **já durante a
terminação**. Contagens refeitas por script sobre o arquivo:

- 8346 linhas, 33 312 bytes recebidos no trecho;
- `80 00 82 8A 90` (comando 0, dado 130): **2776** ocorrências, uma a cada ~50 ms;
- `80 05 00 C4 AF` (comando 5, dado 0): 2776;
- `80 12` (consulta ReadSpeed M1): 2776;
- nenhum outro byte no trecho;
- CRC16 (poly 0x1021, init 0) recalculado: `80 00 82` → `8A90`, `80 05 00` → `C4AF`.

Contadores do próprio emulador: de `com velocidade >127=406` (22:29:18) a
`=3105` (22:31:33), sempre com `CRC ruim=0` e `cmd desconhecido=0`.

## Limites desta evidência

- O trecho começa com a terminação já ativa: a transição (dado ≤127 → 130) e as
  linhas `>> PACOTE DRIVE VALIDO` do início não estão neste arquivo.
- Faltam aqui, por estarem só na máquina da autora: captura completa do Monitor,
  `dmesg`, `work_queue status`, `param show RBCLW_*` e o arquivo
  `correcao_init_roboclaw.patch`.
- Só o canal 1 foi configurado com failsafe fora da faixa.
- O Bug 2 (`write()` em `size_t`) não foi exercitado em hardware.

## Teste de controle: `RBCLW_FAIL1 = 254` (2026-11-12, 23:16)

Mesma montagem e mesmo firmware, mudando só o parâmetro para 254, o maior valor
que ainda resulta em byte válido ((254 − 127)/127 = 1,0 → 127). Com
`actuator_armed: armed: True, termination: True`, a placa passou a enviar

    80 00 7F B4 22

comando 0, **dado `0x7F` = 127**, CRC `B422` (recalculado). Antes da terminação
o tráfego era `80 01 00 08 6B` (comando 1, dado 0). Trecho em
`roboclaw_esp32_captura_controle_254_trecho.txt` (4 pacotes; as linhas cortadas
no início e no fim da colagem foram omitidas).

| `RBCLW_FAIL1` | Byte de dados na terminação | Dentro de 0–127? |
|---|---|---|
| 254 | 127 (`0x7F`) | sim |
| 257 | 130 (`0x82`) | **não** |

Pendente neste controle: a captura completa, com a transição e a linha
`[resumo v4]` (contador `>127`).

## Ressalva sobre os ACKs

Nas duas execuções o `dmesg` da placa registra `ACK timeout` e
`Error reading encoders`, e o intervalo entre pacotes (~12 ms) coincide com o
tempo de espera do driver. O emulador envia as respostas, mas não está
confirmado que a placa as aceita (fio de retorno ou tempo de resposta). Isso
não afeta a medição, que é o byte transmitido pela placa e lido direto do fio;
significa, porém, que o ensaio observa o driver no caminho de erro de resposta.
