# Sensor de vento FT7 na Pixhawk 6X RT: antes e depois da correção

Aprovado pelo orientador em 22/11/2026. Mesma bancada do RoboClaw (11/11):
Pixhawk 6X RT + ESP32 na TELEM2, sem motor e sem hélice.

O ESP32 faz o papel do sensor (emulador `ft7_emulador_esp32/ft7_emulador_esp32.ino`):
responde às consultas da placa com uma leitura normal (12,3 m/s) e, quando você
digita `l` no Monitor serial, envia UMA linha de 48 bytes (46 vírgulas + CR LF),
maior que o buffer de 30 bytes do driver.

Previsão por leitura do código (NÃO medida): a linha longa escreve por cima de
`_interval`, e o driver deixa de consultar o sensor e de publicar `sensor_airflow`.
No texto da dissertação só entra o que for medido.

## Regra de ouro da noite
Depois de CADA passo, copie o texto da tela para um arquivo no Bloco de Notas
e salve na hora (em 11/11 capturas foram perdidas). Nomes sugeridos:
`ft7_antes_01_status.txt`, `ft7_antes_02_listener.txt` ... e `ft7_depois_...`.

## Parte 0 — preparar o ESP32 (uma vez)
1. Arduino IDE → abrir `testes_hardware/ft7/ft7_emulador_esp32/ft7_emulador_esp32.ino`.
2. Placa: "ESP32 Dev Module"; escolher a porta; Upload (se travar em
   "Connecting...", segure o botão BOOT do ESP32 até começar).
3. Abrir o Monitor serial em **115200**. Deve aparecer "EMULADOR FT7 (ESP32) v1".
4. Fios (iguais ao RoboClaw): TX da TELEM2 (amarelo) → GPIO16; RX da TELEM2
   (azul) → GPIO17; GND (preto) → GND. Tudo em 3,3 V.

## Parte 1 — ANTES (firmware atual da placa, driver original)
1. Ligue só a Pixhawk no USB; abra o QGroundControl; menu Analyze Tools → MAVLink Console.
2. Confirme que o driver está no firmware: digite `ft7_technologies status`.
   - Se responder `ft7_technologies: not running`, o driver existe (ótimo).
   - Se disser `command not found`, o firmware não tem o driver: pare e me avise.
3. Em Parameters (QGC), ajuste:
   - `RBCLW_SER_CFG` = 0 (o RoboClaw não pode usar a TELEM2)
   - `SENS_FTX_CFG` = 102 (TELEM2)
   Salve. No console: `param save` e depois `reboot`.
4. Depois do reboot, no console: `ft7_technologies status`
   Esperado: aparece a porta. No Monitor serial do ESP32: o contador de
   "consultas da placa" sobe (cerca de 10 por segundo).
   Salve os dois textos.
5. No console: `listener sensor_airflow`. Esperado: velocidade 12,3.
   Depois: `work_queue status`. Salve os dois.
6. PROVOCAR: no Monitor serial do ESP32, digite `l` e Enter (uma vez).
7. Espere uns 20 segundos e registre, nesta ordem:
   a) Monitor serial do ESP32: as consultas da placa continuam subindo?
   b) Console: `ft7_technologies status`
   c) Console: `listener sensor_airflow` (ainda atualiza? o timestamp avança?)
   d) Console: `work_queue status` e `dmesg`
   e) A placa travou ou reiniciou? Anote o que viu (e a hora).
8. Anote o `md5sum` do firmware que está na placa, se ainda tiver o arquivo
   (`md5sum caminho/do/arquivo.px4`).

## Parte 2 — DEPOIS (com a correção do PX4)
No WSL, na pasta do PX4 (se não lembra onde: `find ~ -maxdepth 3 -name PX4-Autopilot -type d`):
```
cd CAMINHO_DO_PX4
git status --short
cp ~/dissertacao-px4-esbmc/testes_hardware/ft7/correcao_upstream_pr28600.patch /tmp/
git apply --check /tmp/correcao_upstream_pr28600.patch && echo "patch ok"
git apply /tmp/correcao_upstream_pr28600.patch
git diff src/drivers/wind_sensor | head -40
make px4_fmu-v6xrt_default
```
- O `git apply --check` precisa imprimir `patch ok`. Se der erro, pare e me mande a mensagem.
- A compilação leva de 5 a 20 minutos. Ao terminar:
```
ls -la build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.px4
md5sum build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.px4
cp build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.px4 /mnt/c/Users/Public/Documents/px4_ft7_corrigido.px4
```
Anote o md5. Gravar na placa (no QGC):
1. Feche o Monitor serial; **desconecte o USB do ESP32** e o da Pixhawk.
2. QGC → Vehicle Setup → Firmware. Com essa tela aberta, conecte o USB da Pixhawk.
3. Advanced settings → "Custom firmware file..." → escolha `px4_ft7_corrigido.px4` → OK.
4. Espere terminar e a placa reiniciar. NÃO pule esta etapa (em 11/11 a placa
   continuou com o firmware antigo por isso).
5. Reconecte o ESP32 e repita os passos 1 a 8 da Parte 1, trocando o nome dos
   arquivos para `ft7_depois_...`. Os parâmetros podem ter sido apagados:
   confira `SENS_FTX_CFG` = 102 e `RBCLW_SER_CFG` = 0 de novo.

## Como me enviar
Mande os textos na ordem (antes e depois), as saídas do `md5sum`, o `git diff`
do patch e uma foto da montagem. Eu escrevo o resultado só com o que vier deles.

## Cuidados
- ESP32 e Pixhawk só em 3,3 V. Sem hélice, sem motor.
- O sensor FT7 de verdade só deve ser ligado depois, e somente se o datasheet
  confirmar saída UART em 3,3 V (alguns modelos usam RS-232 ou RS-485, com
  tensões que podem danificar a porta). Para esta noite, use só o ESP32.
