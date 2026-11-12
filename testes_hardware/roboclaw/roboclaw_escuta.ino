/*
 * roboclaw_escuta.ino
 *
 * Valida em hardware o Bug 1 do driver roboclaw do PX4
 * (src/drivers/roboclaw/Roboclaw.cpp, commit 8be2590c15): na terminacao
 * de voo, com RBCLW_FAIL1 = 257, a placa ENVIA ao RoboClaw um byte de
 * velocidade acima de 127 (o manual do RoboClaw so admite 0..127).
 *
 * Este programa faz o Arduino de "RoboClaw de mentira": ele SO ESCUTA o
 * que a Pixhawk transmite e avisa quando o byte de velocidade passa de 127.
 *
 * Protocolo (Packet Serial). Para os comandos de acionamento o pacote e:
 *   [endereco][comando][velocidade][crc_hi][crc_lo]
 *   comando = 0 (frente M1), 1 (re M1), 4 (frente M2), 5 (re M2)
 * O byte logo depois do comando e a velocidade (0..127 valido).
 *
 * LIGACAO (so escuta, SEGURO, NAO precisa de conversor de nivel):
 *   - USB do Arduino  -> computador (monitor serial)
 *   - TX da porta serial da Pixhawk  -> pino 0 (RX1) do Arduino
 *   - GND da Pixhawk  -> GND do Arduino
 *   NAO ligue nada no RX da Pixhawk. Nada do Arduino vai para a placa.
 *
 * Endereco padrao do RoboClaw no PX4: 128 (0x80). Se o parametro
 * RBCLW_ADDRESS estiver diferente, ajuste ENDERECO abaixo.
 *
 * Baud: ajuste conforme RBCLW_BAUD. O exemplo usa 38400.
 */

const uint8_t ENDERECO = 0x80;   // RBCLW_ADDRESS (128 por padrao)
const long    BAUD     = 38400;  // tem que bater com RBCLW_BAUD da placa

uint8_t b2 = 0, b1 = 0;          // dois bytes anteriores

bool ehComandoDrive(uint8_t c) {
  return (c == 0 || c == 1 || c == 4 || c == 5);
}

void setup()
{
  Serial.begin(115200);  // monitor (USB)
  Serial1.begin(BAUD);   // escuta a Pixhawk (pino 0 = RX1)
  delay(500);
  Serial.println("Escuta RoboClaw iniciada. Aguardando bytes da Pixhawk...");
  Serial.println("Formato: cada byte em hex; quando achar velocidade, mostra o valor.");
}

void loop()
{
  if (Serial1.available()) {
    uint8_t b = Serial1.read();

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
