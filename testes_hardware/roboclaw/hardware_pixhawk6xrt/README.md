# Validação em hardware do Bug 1 do driver RoboClaw — Pixhawk 6X RT + ESP32

Data dos ensaios: noite de 2026-11-11 (22:29–23:57), com registros finais na madrugada de 2026-11-12. Este registro substitui a conclusão da primeira
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
Esse travamento é um terceiro defeito, observado só neste ensaio e não
verificado formalmente.

O efeito ficou registrado no console da placa, em colagens feitas pela autora
durante a sessão e recuperadas do histórico da conversa em 2026-11-12:

| Arquivo | Driver | Mensagens do driver no boot | `wq:hp_default` em `work_queue status` |
|---|---|---|---|
| `console_original_travado_1.txt` | original | nenhuma depois de `Starting Roboclaw Driver on /dev/ttyS5` | `battery_status` 0,1 Hz (intervalo 11,4 s), `board_adc` 0,3 Hz (3,07 s, esperado 10 ms), `pwm_out` intervalo 62,8 s (esperado 300 ms), `roboclaw` 0,0 Hz |
| `console_original_travado_2.txt` | original, outro boot | nenhuma | `battery_status` 0,1 Hz (14,4 s), `board_adc` 0,2 Hz (3,88 s), `pwm_out` 79,4 s, `roboclaw` 0,0 Hz |
| `console_com_init_funcionando.txt` | com as duas linhas | `Successfully connected` | `battery_status` 99,9 Hz, `board_adc` 99,8 Hz, `pwm_out` 3,4 Hz, `roboclaw` 3,4 Hz |

As outras filas de trabalho mantêm a taxa normal nos três boots (por exemplo
`wq:rate_ctrl` a ~808 Hz). Nos dois boots com o driver original aparece ainda
um item `hardfault_stream` em `wq:hp_default`, ausente no terceiro; não foi
investigado. Os dois primeiros boots são anteriores à primeira proposta da
alteração (01:46Z); o terceiro é posterior à compilação das 21:55 (hora
local). Nos três, `RBCLW_FUNC1` ainda não estava configurado (no terceiro,
`param show` mostra `RBCLW_FUNC1 = 0`).

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

## Foto da montagem

`foto_montagem_pixhawk6xrt_esp32.jpg` (enviada pela autora, girada 180° para
leitura): Pixhawk 6X RT alimentada pela USB, conector na porta TELEM2 e os
fios até o ESP32, que aparece com a face dos componentes para baixo, de modo
que a serigrafia dos pinos não fica visível. A correspondência dos fios
(amarelo TX → GPIO16, azul RX → GPIO17, preto GND) é a descrita em
"Configuração"; a foto mostra a montagem, mas não permite conferir os pinos.

## Limites desta evidência

- O trecho começa com a terminação já ativa: a transição (dado ≤127 → 130) e as
  linhas `>> PACOTE DRIVE VALIDO` do início não estão neste arquivo.
- Faltam aqui, por estarem só na máquina da autora: captura completa do Monitor,
  `dmesg`, `work_queue status`, `param show RBCLW_*` e o arquivo
  `correcao_init_roboclaw.patch`.
- Só o canal 1 foi configurado com failsafe fora da faixa.
- O Bug 2 (`write()` em `size_t`) não foi exercitado em hardware.

## Teste de controle: `RBCLW_FAIL1 = 254` (2026-11-11, 23:16)

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

## Captura longa do ensaio original e transição ao vivo para 254

`roboclaw_esp32_captura_257_longa_com_transicao_254.txt` (78 901 linhas,
22:42:04–23:02:57; a primeira e a última linha vieram cortadas na cópia) é a
continuação da mesma execução de `roboclaw_esp32_captura_trecho.txt`, sem
reinício do ESP32 (os contadores do emulador seguem crescendo).

| Trecho | Pacote do motor 1 | Ocorrências |
|---|---|---|
| 22:42:04 – 22:59:53 | `80 00 82 8A 90` (dado 130) | 21 360 |
| 22:59:53 – 22:59:54 | `80 00 7F B4 22` (dado 127) | 21 |
| 23:00:00 – 23:02:57 | `80 01 00 08 6B` (repouso, após reinício da placa) | 4 914 |

O contador do emulador vai de `>127=15761` (22:42:07) a `>127=37072`
(22:59:55) e para de crescer; `CRC ruim=0` e `cmd desconhecido=0` do começo
ao fim. Somando as 3105 contagens já registradas no primeiro arquivo e o
intervalo entre os dois, o emulador contou 37 072 pacotes com dado acima de
127 em cerca de 30 minutos de terminação contínua.

A troca de 130 para 127 às 22:59:53 acontece dentro da mesma execução, sem
reinício, e coincide com a mudança de `RBCLW_FAIL1` para 254 feita no console
antes do teste de controle (o horário do `param set` não ficou registrado; a
associação é inferida). É a transição que faltava no controle: o mesmo
firmware, na mesma terminação, passa de byte inválido a byte válido quando só
o parâmetro muda. Fora do padrão há três linhas: um byte `00` isolado às
22:59:54 (reinício da placa) e duas linhas embaralhadas pela cópia.

## Antes e depois da correção (2026-11-11, 23:53)

Mesma montagem, mesmo parâmetro do ensaio original (`RBCLW_FAIL1 = 257`,
conferido no console), firmware recompilado com `../correcao_roboclaw.patch`
aplicado sobre o mesmo commit (mantidas as duas linhas de inicialização do
`Roboclaw.hpp`). Arquivo gravado: `px4_roboclaw_CORRIGIDO.px4`
(md5 `69170b2af0299125a1fc3440097c7818`; o firmware anterior,
`px4_roboclaw_fix.px4`, tem md5 `53c54c1d34b1d02fd108726a1d62b0c5`).

Com `actuator_armed: armed: True, termination: True`, a placa enviou

    80 00 7F B4 22

comando 0, **dado `0x7F` = 127**, CRC `B422`. Trecho transcrito da tela do
Monitor em `roboclaw_esp32_captura_257_corrigido_trecho.txt` (5 pacotes).

| Firmware | `RBCLW_FAIL1` | Pacote na terminação | Dado |
|---|---|---|---|
| original (+2 linhas de inicialização) | 257 | `80 00 82 8A 90` | 130, fora da faixa |
| com `correcao_roboclaw.patch` | 257 | `80 00 7F B4 22` | 127, saturado no máximo válido |

Na mesma sessão, minutos antes (≈23:40), a placa ainda estava com o firmware
anterior por engano e, com o mesmo parâmetro, voltou a enviar `80 00 82 8A 90`
— uma segunda reprodução do defeito, em outra sessão de bancada.

Captura completa em `roboclaw_esp32_captura_257_corrigido.txt` (15 827 linhas,
23:53:17–23:57:40, a última cortada na cópia): **5267 pacotes
`80 00 7F B4 22` e nenhum outro pacote de comando 0 ou 1**. O contador do
emulador fica em `>127=0` nas 88 linhas de resumo, de 7007 a 17 444 pacotes
de drive válidos. Há `CRC ruim=1`, constante do primeiro ao último resumo:
ocorreu antes do início da captura, no momento em que o fio de dados foi
reencaixado com a placa já transmitindo. A captura começa com a terminação
já ativa, então não contém a transição a partir do repouso.

Um print de tela das 00:08:24 de 12/11/2026 (relógio do Windows visível;
guardado pela autora) mostra a mesma execução ainda em curso, com
`t=1098…ms` no emulador (cerca de 18 minutos desde o reset do ESP32) e o
pacote do motor 1 ainda em `80 00 7F B4 22`.

`ver_all_firmware_corrigido.txt` traz o `ver all` da placa nesse ensaio: o
`PX4 git-hash` é o commit analisado (`8be2590c…`). O `Build datetime`
(1º de novembro) é a data em que a árvore foi compilada pela primeira vez; ele
não muda em recompilações incrementais e o hash não marca alterações locais,
então o `ver all` confirma o commit de base, mas **não distingue** o firmware
original do corrigido. Essa distinção vem do md5 dos dois arquivos e do
comportamento observado no fio.

`diff_firmware_corrigido.patch` é o `git diff src/drivers/roboclaw/` da árvore
que gerou o firmware corrigido (correção em `Roboclaw.cpp` mais as duas linhas
de `Roboclaw.hpp`), e `correcao_init_roboclaw.patch` é o diff só das duas
linhas, usado no firmware dos ensaios anteriores. Os dois foram reconstruídos
a partir da saída do `cat` colada pela autora (tabulações restauradas); o
tamanho confere byte a byte com o `ls -la` dos originais (2239 e 511 bytes).
Fora de `src/drivers/roboclaw/`, a árvore tinha duas outras alterações, iguais
nos dois firmwares. Em `boards/px4/fmu-v6xrt/default.px4board` foram
acrescentadas duas linhas (`git diff` colado pela autora):
`CONFIG_DRIVERS_ROBOCLAW=y`, que inclui o driver na compilação (ele não faz
parte da configuração padrão desta placa), e
`CONFIG_DRIVERS_WIND_SENSOR_FT_TECHNOLOGIES=y`, resto de outro ensaio e sem
relação com o RoboClaw. `src/modules/zenoh/Kconfig.topics` (+50 linhas) também
aparece alterado; o conteúdo não foi capturado. A correção do Bug 2 (`write()` com retorno
negativo) está no mesmo patch, mas este ensaio não a exercita.

## Ressalva sobre os ACKs

Em todas as execuções o `dmesg` da placa registra `ACK timeout` e
`Error reading encoders`, e o intervalo entre pacotes (~12 ms) coincide com o
tempo de espera do driver. O emulador envia as respostas, mas não está
confirmado que a placa as aceita (fio de retorno ou tempo de resposta). Isso
não afeta a medição, que é o byte transmitido pela placa e lido direto do fio;
significa, porém, que o ensaio observa o driver no caminho de erro de resposta.
