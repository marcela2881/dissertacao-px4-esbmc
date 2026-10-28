# Teste do RoboClaw (Bug 1) em hardware — guia

Objetivo: observar na Pixhawk o Bug 1 do driver roboclaw (commit
8be2590c15): na terminação de voo, com `RBCLW_FAIL1 = 257`, a placa envia
ao RoboClaw um byte de velocidade acima de 127 (o manual só admite 0..127).
A issue atual reproduziu isso só no PC, com UART emulada; o teste em
hardware mostra o mesmo byte saindo do controlador real.

## Por que este teste é o mais seguro
O Arduino SÓ ESCUTA. Ele lê o que a Pixhawk transmite e não manda nada de
volta. Como nada externo entra na porta da Pixhawk, não há risco para a
placa e NÃO precisa de conversor de nível.

## Material
- Arduino (Uno, Nano ou Micro) com cabo USB.
- 2 fios: um para TX da Pixhawk → RX do Arduino, um para GND → GND.
- Nada mais.

## Ligação
- USB do Arduino → computador.
- TX da porta serial da Pixhawk (ex.: TELEM2) → pino RX do Arduino
  (no Micro é o pino 0 / RX1; no Uno é o pino 0 / RX).
- GND da Pixhawk → GND do Arduino.
- NÃO ligar nada no RX da Pixhawk.

## Passo a passo (na quinta)
1. Programar o Arduino com `roboclaw_escuta.ino` (Arduino IDE → Upload).
   - Antes, conferir na placa dois parâmetros e ajustar no `.ino`:
     - `RBCLW_BAUD` (baud) → variável `BAUD` no sketch
     - `RBCLW_ADDRESS` (endereço, padrão 128) → variável `ENDERECO`
2. No QGroundControl, configurar o rover com RoboClaw:
   - selecionar airframe de rover com RoboClaw (Vehicle Setup → Airframe)
   - atribuir o driver roboclaw à porta serial usada (TELEM2) e o baud
   - `RBCLW_FAIL1 = 257` (o valor que dispara o bug)
   - `SDLOG_MODE = 2` (já configurado)
   (Os nomes exatos dos parâmetros eu confirmo contigo na hora.)
3. Abrir o monitor serial do Arduino IDE (115200 baud) para ver a saída.
4. Reproduzir:
   - armar a placa na bancada (SEM motor, SEM hélice; RoboClaw não ligado)
   - disparar a terminação de voo (ex.: `MAV_CMD_DO_FLIGHTTERMINATION`,
     ou uma ação de failsafe configurada como Terminate)
5. Observar no monitor serial:
   - para cada comando de acionamento, o sketch mostra `velocidade = N`
   - se N > 127, imprime `*** PASSOU DE 127 (BUG CONFIRMADO) ***`
6. Salvar a saída do monitor serial num arquivo de texto (evidência).

## Resultado esperado
Em operação normal (saída limitada a 254), o byte fica ≤ 127. Na
terminação com `RBCLW_FAIL1 = 257`, o sketch mostra um byte acima de 127
(130, pela análise da issue). Essa é a confirmação em hardware.

## Observações
- Bug 2 (erro de `write()` não detectado) NÃO é testável em hardware de
  forma confiável, porque depende de a função `write()` do sistema falhar.
  Fica só com a verificação no PC.
- Teste de bancada: sem motor, sem hélice.
- Reportar ao upstream só depois da revisão do Prof. Lucas.
