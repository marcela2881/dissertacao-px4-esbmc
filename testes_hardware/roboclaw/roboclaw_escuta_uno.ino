/*
 * roboclaw_escuta_uno.ino  -- versao para ARDUINO UNO
 *
 * Valida em hardware o Bug 1 do driver roboclaw do PX4
 * (src/drivers/roboclaw/Roboclaw.cpp, commit 8be2590c15): na terminacao de
 * voo, com RBCLW_FAIL1 = 257, a placa ENVIA ao RoboClaw um byte de velocidade
 * acima de 127 (o manual do RoboClaw so admite 0..127).
 *
 * O Arduino faz de "RoboClaw de mentira": ele SO ESCUTA o que a Pixhawk
 * transmite e avisa quando o byte de velocidade passa de 127.
 *
 * POR QUE ESTA VERSAO E DIFERENTE:
 * O Uno tem apenas UMA serial de hardware (pinos 0/1), que e a mesma do USB.
 * Por isso usamos SoftwareSerial para escutar a Pixhawk num pino separado
 * (pino 10) e deixamos o USB livre para o monitor serial.
 *
 * LIGACAO (so escuta, SEGURO, NAO precisa de conversor de nivel):
 *   - USB do Arduino            -> computador (monitor serial)
 *   - TX da porta serial da Pixhawk -> pino 10 do Arduino
 *   - GND da Pixhawk            -> GND do Arduino
 *   NAO ligue nada no RX da Pixhawk. Nada do Arduino vai para a placa.
 *
 * Endereco padrao do RoboClaw no PX4: 128 (0x80). Se RBCLW_ADDRESS estiver
 * diferente, ajuste ENDERECO abaixo. BAUD tem que bater com RBCLW_BAUD.
 */
#include <SoftwareSerial.h>

const uint8_t ENDERECO = 0x80;   // RBCLW_ADDRESS (128 por padrao)
const long    BAUD     = 38400;  // TELEM2 reiniciada a 38400 (via SER_TEL1_BAUD) - Uno le limpo aqui

const uint8_t PINO_RX = 10;      // TX da Pixhawk entra AQUI
const uint8_t PINO_TX = 11;      // nao usado (so escuta), mas exigido pela lib

SoftwareSerial pixhawk(PINO_RX, PINO_TX);

uint8_t b2 = 0, b1 = 0;          // dois bytes anteriores

bool ehComandoDrive(uint8_t c) {
  return (c == 0 || c == 1 || c == 4 || c == 5);
}

void setup()
{
  Serial.begin(115200);   // monitor (USB)
  pixhawk.begin(BAUD);    // escuta a Pixhawk (pino 10)
  delay(500);
  Serial.println("Escuta RoboClaw iniciada (UNO). Aguardando bytes da Pixhawk...");
  Serial.println("Cada byte aparece em hex; ao achar a velocidade, mostra o valor.");
}

void loop()
{
  if (pixhawk.available()) {
    uint8_t b = pixhawk.read();

    // log cru de todos os bytes
    Serial.print("0x");
    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");

    // deteccao: [ENDERECO][comando drive][velocidade]
    if (b2 == ENDERECO && ehComandoDrive(b1)) {
      Serial.println();
      Serial.print(">> velocidade = ");
      Serial.print(b);
      if (b > 127) {
        Serial.println("  *** PASSOU DE 127 (BUG CONFIRMADO) ***");
      } else {
        Serial.println("  (dentro do limite 0..127)");
      }
    }

    b2 = b1;
    b1 = b;
  }
}
