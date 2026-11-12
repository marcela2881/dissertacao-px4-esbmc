/*
 * ft7_emulador_esp32.ino  (v1)
 *
 * O ESP32 faz o papel do sensor de vento FT Technologies FT7 para o driver
 * ft7_technologies do PX4 (commit 8be2590c15).
 *
 * Protocolo (ft7_technologies.cpp):
 *   placa  -> sensor : "$//,WV?*//\r\n"            (consulta, 12 bytes, ate 10 Hz)
 *   sensor -> placa  : "$WI,WVP=<vel>,<ang>,<st>*<XX>\r\n"
 *   XX = XOR, em hexadecimal, dos bytes entre '$' e '*'.
 *
 * Ligacao (a mesma do ensaio do RoboClaw, ESP32 em 3,3 V, sem conversor):
 *   TX da TELEM2 (amarelo) -> GPIO16 (RX2)
 *   RX da TELEM2 (azul)    -> GPIO17 (TX2)
 *   GND (preto)            -> GND
 *
 * Uso pelo Monitor serial (115200), digitando uma letra e Enter:
 *   n  -> responde cada consulta com uma mensagem normal (padrao ao ligar)
 *   l  -> envia UMA linha longa (46 virgulas + \r\n) e continua em modo normal
 *   p  -> para de responder
 */

const long BAUD_FT7 = 9600;   // o driver fixa B9600
const int  PINO_RX  = 16;
const int  PINO_TX  = 17;

const int  N_VIRGULAS = 46;   // 30 de _readbuf + ate 7 de alinhamento + 8 de _interval + 1

bool responder = true;
unsigned long consultas = 0, respostas = 0, linhasLongas = 0, bytesRx = 0;
unsigned long ultimaConsulta = 0, ultimoResumo = 0;
char   janela[12];
int    nJanela = 0;

void enviaNormal() {
  const char corpo[] = "WI,WVP=123,090,0";          // 12,3 m/s, 90 graus, status 0
  uint8_t x = 0;
  for (unsigned i = 0; i < sizeof(corpo) - 1; i++) x ^= (uint8_t)corpo[i];
  char msg[48];
  snprintf(msg, sizeof(msg), "$%s*%02X\r\n", corpo, x);
  Serial2.print(msg);
  respostas++;
}

void enviaLonga() {
  for (int i = 0; i < N_VIRGULAS; i++) Serial2.write(',');
  Serial2.write('\r');
  Serial2.write('\n');
  linhasLongas++;
  Serial.print(">> LINHA LONGA ENVIADA: ");
  Serial.print(N_VIRGULAS);
  Serial.println(" virgulas + CR LF (48 bytes; o buffer do driver tem 30)");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(BAUD_FT7, SERIAL_8N1, PINO_RX, PINO_TX);
  delay(300);
  Serial.println("=== EMULADOR FT7 (ESP32) v1 ===");
  Serial.println("Comandos: n = normal | l = uma linha longa | p = parar de responder");
}

void loop() {
  // bytes vindos da placa: procura a consulta "$//,WV?*//\r\n"
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    bytesRx++;
    if (c == '$') nJanela = 0;
    if (nJanela < (int)sizeof(janela)) janela[nJanela++] = c;
    if (c == '\n') {
      if (nJanela == 12 && memcmp(janela, "$//,WV?*//\r\n", 12) == 0) {
        consultas++;
        ultimaConsulta = millis();
        if (responder) enviaNormal();
      }
      nJanela = 0;
    }
  }

  // comandos pelo Monitor
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'n') { responder = true;  Serial.println(">> modo normal"); }
    if (c == 'p') { responder = false; Serial.println(">> parado: nao responde mais"); }
    if (c == 'l') { enviaLonga(); }
  }

  // resumo a cada 2 s
  if (millis() - ultimoResumo > 2000) {
    ultimoResumo = millis();
    Serial.print("[ft7 v1] t=");                 Serial.print(millis() / 1000);
    Serial.print("s | consultas da placa=");     Serial.print(consultas);
    Serial.print(" | respostas enviadas=");      Serial.print(respostas);
    Serial.print(" | linhas longas=");           Serial.print(linhasLongas);
    Serial.print(" | bytes recebidos=");         Serial.print(bytesRx);
    Serial.print(" | ultima consulta ha ");
    if (consultas == 0) Serial.println("-- (nenhuma ainda)");
    else { Serial.print((millis() - ultimaConsulta) / 1000.0, 1); Serial.println(" s"); }
  }
}
