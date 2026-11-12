# Nota de validação em hardware — driver RoboClaw (Bug 1)

> **Atualização (2026-11-12):** o Bug 1 **foi observado no fio** num Pixhawk 6X RT
> real, com o RoboClaw emulado por um ESP32: pacote `80 00 82 8A 90` (dado 130)
> durante a terminação de voo. Ver `hardware_pixhawk6xrt/README.md`.
> Os itens 2 e 5 abaixo estavam errados: `SER_TEL2_BAUD` existe (padrão 921600,
> oculto enquanto não usado) e a terminação pode ser disparada na bancada com
> `commander termination`. O texto abaixo fica como registro da primeira tentativa.

Registro do teste de bancada feito num **Pixhawk 6X RT real**, com o que ficou
comprovado, o que **não** ficou, e o que falta para retomar depois. Escrito para
não deixar nenhum número inflado entrar na dissertação.

## Objetivo

Observar, no hardware físico, o driver `roboclaw` transmitindo na serial, para
complementar a prova formal do Bug 1 (byte de velocidade > 127).

## Montagem usada

- **Firmware:** PX4 commit `8be2590c15`, placa `fmu-v6xrt`, driver habilitado
  (`CONFIG_DRIVERS_ROBOCLAW=y`), compilado com `make px4_fmu-v6xrt_default` e
  gravado num **Pixhawk 6X RT**.
- **Porta:** TELEM2 = `/dev/ttyS5` nesta placa (confirmado em
  `boards/px4/fmu-v6xrt/default.px4board`: `CONFIG_BOARD_SERIAL_TEL2="/dev/ttyS5"`).
- **Parâmetros:** `RBCLW_SER_CFG=102` (TELEM2), `RBCLW_ADDRESS=128`,
  `RBCLW_FAIL1=257`, `RBCLW_FUNC1=101`.
- **Leitor:** Arduino Uno escutando o TX da TELEM2 (só leitura), via
  `SoftwareSerial` no pino 10 — sketch `roboclaw_escuta_uno.ino`.

## O que ficou comprovado (honesto)

- O firmware com o driver **compila, grava e roda** num Pixhawk 6X RT real.
- **Há transmissão** na TELEM2 (chegaram bytes ao leitor).

## O que NÃO ficou comprovado, e por quê

1. **Leitura limpa do protocolo:** o Arduino Uno usa `SoftwareSerial`, confiável
   só até ~38400 baud. A TELEM2 ficou num baud mais alto que o Uno não lê sem
   embaralhar.
2. **Baud não ajustável por parâmetro:** nesta build **não existe `SER_TEL2_BAUD`**
   (`param show SER_TEL2*` não retorna nada), então não dá para baixar a TELEM2
   por parâmetro.
3. **Reinício manual não confiável:** `roboclaw stop` deu `timeout, forcing stop`
   e o `roboclaw start /dev/ttyS5 SER_TEL1_BAUD` seguinte não reabriu a porta a
   38400 de forma estável.
4. **Falsos positivos:** com baud errado, o fluxo embaralhado contém, por acaso,
   a sequência `[0x80][comando 0/1/4/5][X]`, o que **disparou alertas espúrios de
   ">127"** no sketch (valores aleatórios como 192 e 34). **Isso NÃO é o bug** —
   é ruído lido errado, e não deve ser usado como evidência.
5. **O byte do bug (130) não aparece na bancada:** ele só é gerado numa
   **terminação de voo real**, que não se força com segurança numa mesa.

## Onde está a prova real do Bug 1

Na verificação **formal + UART emulada**, não no hardware de bancada:

- `harness_roboclaw_bug1.cpp` — ESBMC e CBMC: FAILED sem correção, SUCCESSFUL
  com `-DCORRECAO`.
- `RoboclawTest.cpp` — modelo de RoboClaw em software: o byte **130** sai de fato
  na saída, de forma determinística.
- Logs e hashes em `evidencia_maquina/`.

O hardware de bancada é, no máximo, uma **demonstração de que o driver roda no
silício real** — e ainda assim só mostraria o valor de repouso (127), nunca o 130.

## Para retomar depois (pendente)

Trocar o Arduino Uno por um leitor com **UART de hardware de verdade**, que lê o
baud alto sem embaralhar e elimina os falsos positivos:

- **ESP32** (3.3V, casa com a Pixhawk) — preferido;
- **Arduino Mega 2560** (usa `Serial1`, ver `roboclaw_escuta.ino`);
- **Adaptador USB-TTL direto no PC** (lê qualquer baud; o mais simples).

Com qualquer um deles, o objetivo possível é capturar o **pacote limpo**
(`0x80` + comando + velocidade **127**) — demonstração do driver falando o
protocolo correto. O Bug 1 em si permanece provado pela via formal/emulada.
