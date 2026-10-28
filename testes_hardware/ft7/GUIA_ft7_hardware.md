# Teste do ft7 em hardware — guia

Objetivo: observar na Pixhawk o bug do driver de vento FT7 (candidato 3,
commit 8be2590c15), já corrigido pelo PX4 no PR #28600. O Arduino faz o
papel do sensor e manda uma mensagem comprida demais, que o driver copia
para fora dos buffers `_readbuf[30]` e `_raw_speed[5]`.

## Estado atual (feito nesta sessão)
- Firmware recompilado COM o driver ft7 ligado
  (`CONFIG_DRIVERS_WIND_SENSOR_FT_TECHNOLOGIES=y`). Confirmado dentro do
  `.px4` gerado.
- Programa do Arduino pronto: `sensor_ft7_simulado.ino`.

## Material (chega na quinta)
- Arduino (Uno, Nano ou Micro) SOLTO, com pinos e cabo USB.
- Conversor de nível lógico 5 V / 3,3 V (ou 2 resistores: 1 kΩ e 2 kΩ).
- Cabo da porta serial da Pixhawk (TELEM2) com pontas soltas.

## ATENÇÃO — tensão
O TX do Arduino é 5 V. A porta serial da Pixhawk só aceita 3,3 V.
O fio que vai do **TX do Arduino para o RX da Pixhawk** TEM que passar
pelo conversor de nível. Ligar direto pode queimar a porta da placa.

## Passo a passo (na quinta, com o material)

1. Gravar na placa o firmware com o ft7:
   `C:\Users\Public\Documents\` — vou te passar a cópia atualizada.
2. Programar o Arduino com `sensor_ft7_simulado.ino` (Arduino IDE → abrir →
   selecionar a placa e a porta → Upload).
3. Ligar os fios (vou te passar o esquema exato conforme a porta e o
   conversor que chegarem). Em resumo:
   - Arduino GND → Pixhawk GND
   - Arduino TX (pino 1) → conversor → Pixhawk RX
4. No QGroundControl, apontar o driver para a porta usada:
   - Parameters → `SENS_EN_FT7` (ou equivalente) = ativar
   - mapear o ft7 para a porta serial (TELEM2), via `SER_TEL2_BAUD` = 9600
     e o parâmetro de atribuição do sensor.
   (O passo exato de parâmetro eu confirmo na hora.)
5. Deixar a placa gravando log (`SDLOG_MODE = 2`, já configurado).
6. Observar o resultado:
   - Sinal esperado do bug: a placa reinicia, trava, ou o driver reporta
     erro/registra dado corrompido ao receber a mensagem longa.
   - Baixar o log e, se houver, registrar a saída do console.

## Resultado esperado
Com as mensagens normais, o driver publica `sensor_airflow` sem problema.
Com a mensagem longa, o driver escreve além de `_readbuf[30]` /
`_raw_speed[5]` — comportamento indefinido no alvo real. É a confirmação
em hardware do que o ESBMC/CBMC/sanitizers já mostraram e o PR #28600
corrigiu.

## Importante
- Teste de bancada: sem hélices, sem motores.
- É um EXTRA. A validação que a defesa exige é a do GPS.
