/*
 * roboclaw_emulador_esp32.ino  -- ESP32 faz o papel do RoboClaw
 *
 * POR QUE EMULAR: o driver do PX4 (src/drivers/roboclaw/Roboclaw.cpp, commit
 * 8be2590c15) pergunta o status ao RoboClaw ao iniciar (comando 90) e SE ENCERRA
 * se nao receber 6 bytes de resposta ("No valid response, stopping driver").
 * So escutando, o driver nunca chega a mandar comando de motor. Este sketch
 * responde como um RoboClaw e registra o byte de velocidade que a Pixhawk envia.
 *
 * O QUE ELE RESPONDE (tirado do codigo do driver):
 *   consulta [end][cmd]                      -> dados zerados + CRC16
 *       90 (ReadStatus)          resposta de  6 bytes
 *       18, 19 (ReadSpeed M1/M2) resposta de  7 bytes
 *       78 (ReadEncoderCounters) resposta de 10 bytes
 *   comando  [end][cmd][dados][CRC_hi][CRC_lo] -> 0xFF (ACK), se o CRC fechar
 *       0, 1, 4, 5 (drive)       1 byte de dados  <-- o byte do Bug 1
 *       32, 33 (duty cycle)      2 bytes de dados
 *       20 (reset encoders)      sem dados
 *   CRC16: poly 0x1021, init 0, sobre [end][cmd][dados] (igual ao _calcCRC do driver).
 *
 * LIGACAO (agora sao TRES fios; 3.3V dos dois lados, sem conversor de nivel):
 *   TELEM2 pino 2 (TX)  -> GPIO16 (RX2) do ESP32
 *   TELEM2 pino 3 (RX)  <- GPIO17 (TX2) do ESP32      <-- fio NOVO
 *   TELEM2 pino 6 (GND) -- GND do ESP32
 *   NAO ligue o pino 1 (5V) da TELEM2.
 *
 * Monitor Serial: 115200.
 */

const uint8_t ENDERECO = 0x80;   // RBCLW_ADDRESS
const long    BAUD     = 38400;  // SER_TEL1_BAUD = 38400 (conferido no NSH)

const int PINO_RX = 16;          // GPIO16 (RX2) <- TX da TELEM2
const int PINO_TX = 17;          // GPIO17 (TX2) -> RX da TELEM2

uint8_t  rx[8];
int      n = 0;                  // bytes ja recebidos do pacote atual
int      esperado = 0;           // tamanho total do pacote atual
unsigned long ultimoByte = 0;

unsigned long bytesBrutos = 0;
uint8_t  cru[256];
int      cruN = 0;
unsigned long inicioRajada = 0;
unsigned long consultas = 0, drives = 0, crcRuins = 0, acima127 = 0, desconhecidos = 0;
int  ultimaVel[6] = { -1, -1, -1, -1, -1, -1 };   // ultima velocidade vista por comando
bool viuStatus = false;
unsigned long ultimoResumo = 0;

uint16_t crc16(const uint8_t *d, int len, uint16_t crc = 0) {
  for (int i = 0; i < len; i++) {
    crc ^= (uint16_t)d[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

// tamanho da resposta (com CRC) para comandos de consulta; 0 se nao for consulta
int tamanhoResposta(uint8_t cmd) {
  if (cmd == 90) return 6;
  if (cmd == 18 || cmd == 19) return 7;
  if (cmd == 78) return 10;
  return 0;
}

// tamanho total do pacote (com end, cmd e CRC) para comandos com ACK; 0 se desconhecido
int tamanhoComando(uint8_t cmd) {
  if (cmd == 0 || cmd == 1 || cmd == 4 || cmd == 5) return 5;
  if (cmd == 32 || cmd == 33) return 6;
  if (cmd == 20) return 4;
  return 0;
}

void respondeConsulta(uint8_t cmd, int tam) {
  uint8_t resp[10] = {0};
  uint8_t cab[2] = { ENDERECO, cmd };
  uint16_t crc = crc16(cab, 2);
  crc = crc16(resp, tam - 2, crc);          // dados zerados
  resp[tam - 2] = crc >> 8;
  resp[tam - 1] = crc & 0xFF;
  Serial2.write(resp, tam);                  // responde ANTES de qualquer print
  consultas++;
  if (cmd == 90 && !viuStatus) {
    viuStatus = true;
    Serial.println(">> Driver perguntou o status (cmd 90) e foi respondido. Ele deve seguir rodando.");
  }
}

void trataComando() {
  uint16_t crcCalc = crc16(rx, esperado - 2);
  uint16_t crcRx = ((uint16_t)rx[esperado - 2] << 8) | rx[esperado - 1];
  if (crcCalc != crcRx) { crcRuins++; return; }

  Serial2.write((uint8_t)0xFF);              // ACK primeiro

  uint8_t cmd = rx[1];
  if (cmd == 0 || cmd == 1 || cmd == 4 || cmd == 5) {
    uint8_t vel = rx[2];
    drives++;
    bool mudou = (ultimaVel[cmd] != vel);
    ultimaVel[cmd] = vel;
    if (vel > 127) acima127++;
    // registra quando o valor muda, e sempre nos primeiros casos acima de 127
    if (mudou || (vel > 127 && acima127 <= 20)) {
      Serial.print(">> PACOTE DRIVE VALIDO (CRC ok) | comando=");
      Serial.print(cmd);
      Serial.print(" | velocidade=");
      Serial.print(vel);
      Serial.print(" | hex:");
      for (int i = 0; i < 5; i++) { Serial.print(rx[i] < 16 ? " 0" : " "); Serial.print(rx[i], HEX); }
      if (vel > 127) Serial.println("   *** >127 : BUG 1 OBSERVADO NO FIO ***");
      else           Serial.println("   (dentro de 0..127)");
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.setRxFIFOFull(1);                            // entrega cada byte na hora (sem esperar o FIFO encher)
  Serial2.begin(BAUD, SERIAL_8N1, PINO_RX, PINO_TX);
  Serial2.setRxFIFOFull(1);
  Serial2.setRxTimeout(1);                             // 1 simbolo de espera: resposta dentro dos 11 ms do driver
  delay(300);
  Serial.println("=== EMULADOR RoboClaw (ESP32) v4 ===");
  Serial.print("Baud: "); Serial.print(BAUD);
  Serial.print(" | Endereco: 0x"); Serial.println(ENDERECO, HEX);
  Serial.println("Responde ao driver e registra o byte de velocidade. Aguardando o driver...");
}

void loop() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    ultimoByte = millis();
    bytesBrutos++;
    if (cruN == 0) inicioRajada = ultimoByte;
    if (cruN < (int)sizeof(cru)) cru[cruN++] = b;   // guarda; imprime depois, para nao atrasar a resposta

    if (n == 0) {
      if (b == ENDERECO) { rx[0] = b; n = 1; }
      continue;
    }

    if (n == 1) {
      rx[1] = b;
      int tr = tamanhoResposta(b);
      if (tr > 0) { respondeConsulta(b, tr); n = 0; continue; }
      esperado = tamanhoComando(b);
      if (esperado == 0) {                    // comando que nao conhecemos: ressincroniza
        desconhecidos++;
        n = (b == ENDERECO) ? 1 : 0;
        continue;
      }
      n = 2;
      if (n == esperado) { trataComando(); n = 0; }
      continue;
    }

    rx[n++] = b;
    if (n == esperado) { trataComando(); n = 0; }
  }

  // despejo dos bytes crus: quando a linha fica 8 ms parada ou o buffer enche
  if (cruN > 0 && (millis() - ultimoByte > 8 || cruN >= (int)sizeof(cru))) {
    Serial.print("[rx t="); Serial.print(inicioRajada); Serial.print("ms n="); Serial.print(cruN); Serial.print("]");
    for (int i = 0; i < cruN; i++) { Serial.print(cru[i] < 16 ? " 0" : " "); Serial.print(cru[i], HEX); }
    Serial.println();
    cruN = 0;
  }

  // pacote pela metade ha mais de 20 ms: descarta
  if (n > 0 && millis() - ultimoByte > 20) n = 0;

  // resumo a cada 3 s
  if (millis() - ultimoResumo > 3000) {
    ultimoResumo = millis();
    Serial.print("[resumo v4] bytes recebidos="); Serial.print(bytesBrutos);
    Serial.print(" | consultas respondidas="); Serial.print(consultas);
    Serial.print(" | pacotes drive validos=");       Serial.print(drives);
    Serial.print(" | com velocidade >127=");         Serial.print(acima127);
    Serial.print(" | CRC ruim=");                    Serial.print(crcRuins);
    Serial.print(" | cmd desconhecido=");            Serial.println(desconhecidos);
  }
}
