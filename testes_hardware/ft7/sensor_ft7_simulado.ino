/*
 * sensor_ft7_simulado.ino
 *
 * Faz o Arduino se passar pelo sensor de vento FT Technologies (FT7),
 * para validar em hardware o driver ft7_technologies do PX4
 * (commit 8be2590c15), candidato 3 da dissertacao.
 *
 * O driver le cada byte recebido e guarda em _readbuf[30] e em
 * _raw_speed[5] SEM checar o tamanho (ft7_technologies.cpp, commit
 * 8be2590c15). Uma mensagem com campo de velocidade longo demais, ou
 * comprida demais antes do '\n', faz o driver escrever fora desses
 * buffers. O PX4 corrigiu isso no PR #28600.
 *
 * Formato esperado pelo driver:
 *   $<id>,WVP=<speed>,<angle>,<status>*<checksum><cr><lf>
 *
 * LIGACAO (Arduino Micro):
 *   - USB do Arduino  -> computador (alimenta e mostra o monitor serial)
 *   - pino 1 (TX1)    -> conversor de nivel -> RX da porta serial da Pixhawk
 *   - pino 0 (RX1)    -> (opcional) TX da Pixhawk
 *   - GND             -> GND da Pixhawk
 *   ATENCAO: o TX do Arduino e 5 V; a Pixhawk so aceita 3,3 V.
 *   O fio Arduino->Pixhawk TEM que passar pelo conversor de nivel.
 *
 * Baud: 9600 (o driver usa B9600).
 */

// Mensagem NORMAL, bem formada (caminho feliz): velocidade 12.3, angulo 180, status 0
const char MSG_NORMAL[] = "$WI,WVP=123,1800,0*00\r\n";

// Mensagem MALFORMADA: campo de velocidade com muitos digitos e linha
// comprida, para passar de _raw_speed[5] e de _readbuf[30]. Nenhum '\n'
// no meio, so no fim, entao o contador de bytes cresce sem parar.
const char MSG_LONGA[] = "$WI,WVP=99999999999999999999999999999999999999,1800,0*00\r\n";

void setup()
{
  Serial.begin(9600);    // monitor serial (USB), para voce acompanhar
  Serial1.begin(9600);   // vai para a Pixhawk (pinos 0 e 1)
  delay(1000);
  Serial.println("Simulador FT7 iniciado. Enviando mensagens para a Pixhawk.");
}

void loop()
{
  // 3 mensagens normais (mostram o driver funcionando)
  for (int i = 0; i < 3; i++) {
    Serial1.print(MSG_NORMAL);
    Serial.println("enviado: normal");
    delay(500);
  }

  // 1 mensagem malformada (deve disparar o estouro no driver)
  Serial1.print(MSG_LONGA);
  Serial.println("enviado: MALFORMADA (longa)");
  delay(500);
}
