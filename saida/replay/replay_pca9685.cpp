/*
 * REPLAY (teste de reprodução) — PCA9685, contraexemplo T3 (DS1)
 *
 * Compila o PCA9685.cpp ORIGINAL do PX4 com o g++ comum e EXECUTA
 * updateFreq() + calcRawFromPulse() com os valores do contraexemplo e com
 * uma varredura de frequências/pulsos, mostrando a contagem de 12 bits que
 * seria escrita no chip (limite do contador: 4095; 4096 = full ON).
 * O barramento I2C é simulado: toda transferência devolve sucesso (0).
 */
#include <cstdio>
#include <cstdint>
#include <cmath>

extern "C" int nondet_int() { return 0; }          /* I2C: sucesso */
extern "C" uint8_t nondet_uint8() { return 0; }

#include "PCA9685.h"
#include "PCA9685.cpp"     /* arquivo ORIGINAL do PX4 */

uint8_t  g_last_send[STUB_I2C_MAX_SEND];
unsigned g_last_send_len = 0;

using namespace drv_pca9685_pwm;

class PCA9685Replay : public PCA9685
{
public:
	PCA9685Replay() : PCA9685(1, 0x40) {}
};

int main()
{
	int violacoes = 0;
	printf("=== Contraexemplo T3 (ESBMC): freq = 1524.39 Hz, pulso = 1024 us ===\n");
	{
		PCA9685Replay dev;
		dev.updateFreq(1524.39f);
		uint16_t raw = dev.calcRawFromPulse(1024);
		printf("freq efetiva = %.2f Hz, PRE_SCALE escrito = %u, contagem = %u (%s)\n",
		       (double)dev.getFreq(), g_last_send[1], raw, raw > 4096 ? "VIOLA > 4096" : "ok");
		violacoes += raw > 4096;
		/* o que vai para o chip: writePWM grava (raw >> 8) em LEDn_OFF_H; o bit 4 (0x10)
		 * desse registrador é "full OFF" (PCA9685_LED_ON_FULL_ON_OFF_MASK no driver) */
		dev.updateRAW(&raw, 1);
		uint8_t off_h = g_last_send[4];
		printf("registrador LED0_OFF_H escrito = 0x%02X -> bit 4 (full OFF) %s\n", off_h,
		       (off_h & 0x10) ? "LIGADO: o canal fica DESLIGADO (sem pulso)" : "desligado");
	}

	printf("\n=== Varredura: maior pulso padrão (2200 us) por frequência ===\n");
	printf("(documentação de PCA9685_PWM_FREQ: não passar de 400 Hz com saídas de pulso 1000-2000 us)\n");
	printf("%-10s %-10s %-10s %-9s %s\n", "freq (Hz)", "freq ef.", "contagem", "OFF_H", "situação");
	const float freqs[] = {50.f, 200.f, 400.f, 450.f, 600.f, 1000.f, 1525.f};

	for (float f : freqs) {
		PCA9685Replay dev;
		dev.updateFreq(f);
		uint16_t raw = dev.calcRawFromPulse(2200);
		dev.updateRAW(&raw, 1);
		uint8_t off_h = g_last_send[4];
		printf("%-10.1f %-10.2f %-10u 0x%02X      %s\n", (double)f, (double)dev.getFreq(), raw, off_h,
		       raw > 4096 ? ((off_h & 0x10) ? "VIOLA > 4096; canal DESLIGADO (full OFF)" : "VIOLA > 4096; pulso errado (bits altos fora do contador)")
		       : "ok");
	}

	printf("\nRESUMO REPLAY PCA9685: contraexemplo T3 %s\n", violacoes ? "REPRODUZIDO" : "NÃO reproduzido");
	return 0;
}
