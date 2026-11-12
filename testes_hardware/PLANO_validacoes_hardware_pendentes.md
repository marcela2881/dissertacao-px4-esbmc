# Validações em hardware ainda por fazer (plano de 2026-11-12)

O orientador pediu pelo menos duas validações em hardware. Situação:

| Módulo | Estado | O que o hardware mostra |
|---|---|---|
| GPS (placa → GPS) | feito em 27–28/10/2026 (`validacao_gps_hardware_2026-10/`) | o estado do contraexemplo não ocorre no driver de produção |
| RoboClaw, Bug 1 | feito em 11/11/2026 (`roboclaw/hardware_pixhawk6xrt/`) | o defeito ocorre na placa e a correção funciona |
| FT7 (sensor de vento) | **adiado** | — |
| GPS (GPS → placa) | **adiado** | — |

A confirmar com o orientador: se a validação do GPS de outubro conta como uma
das duas.

## Decisão sobre a ligação

Não usar pino macho na porta GPS1 (conector JST-GH, passo de 1,25 mm; risco de
curto com o 5 V e de danificar os contatos). Os dois ensaios pendentes usam a
TELEM2, com o cabo e o ESP32 já montados para o RoboClaw
(amarelo TX → GPIO16, azul RX → GPIO17, preto GND). Um ensaio por vez.

## FT7 — pronto para executar (cerca de 1 hora)

Material já no repositório:

- `ft7/ft7_emulador_esp32/ft7_emulador_esp32.ino`: o ESP32 responde às
  consultas da placa com uma leitura válida e, ao receber `l` pelo Monitor,
  envia uma linha de 48 bytes (46 vírgulas + CR LF).
- `ft7/correcao_upstream_pr28600.patch`: a correção adotada pelo PX4
  (commit `3acf7935a0`, de 2026-09-16), para o ensaio de antes e depois.
- O firmware gravado na placa em 11/11 já inclui o driver
  (`CONFIG_DRIVERS_WIND_SENSOR_FT_TECHNOLOGIES=y`).

Passos:

1. Gravar o sketch no ESP32; Monitor em 115200.
2. No console: `param set RBCLW_SER_CFG 0`, `param set SENS_FTX_CFG 102`,
   `param save`, `reboot`.
3. Conferir `ft7_technologies status` e, no Monitor, `consultas da placa`
   subindo (~10 por segundo) com `respostas enviadas` acompanhando;
   `listener sensor_airflow` deve mostrar velocidade 12,3.
4. Digitar `l` no Monitor; registrar o Monitor, `listener sensor_airflow`,
   `ft7_technologies status` e `work_queue status`.
5. Aplicar o patch, recompilar, gravar e repetir os passos 3 e 4.

Previsão, pela leitura do código no commit `8be2590c15` (não medida):
`_readbuf[30]` é seguido por `_interval` (`uint64_t`), `_collect_phase` e
`_fd`. A linha longa escreve por cima de `_interval`; como o driver só volta à
fase de consulta quando o tempo decorrido passa de `1,5 × _interval`, ele
deixaria de enviar consultas e de publicar `sensor_airflow`. Dependendo do
alinhamento, parte de `_fd` também pode ser sobrescrita. A linha usa só
separadores para não tocar em `_raw_speed`/`_raw_angle`, isolando o estouro de
`_readbuf`.

Este defeito não foi reportado pela autora: já estava corrigido no PX4 por
terceiros (PR 28600) quando foi conferido contra o `main`.

## GPS no sentido GPS → placa — por preparar (cerca de 1 hora)

Ideia: `GPS_1_CONFIG = 102` (TELEM2), `GPS_DUMP_COMM = 1`, `SDLOG_MODE = 2`;
o ESP32 envia bytes em rajadas de tamanhos variados; analisar `gps_dump` com
pyulog, como no ensaio de outubro, conferindo que `len` (sem o bit 7) nunca
passa de 79. O sketch do ESP32 ainda não foi escrito. A fonte dos bytes seria
emulada, não um receptor u-blox real, e isso deve ser declarado.
