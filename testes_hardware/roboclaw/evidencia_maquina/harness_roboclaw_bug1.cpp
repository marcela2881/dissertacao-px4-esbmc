/*
 * harness_roboclaw_bug1.cpp
 * RoboClaw - Bug 1: o byte de velocidade pode passar de 127.
 *
 * MODULO PX4: src/drivers/roboclaw/Roboclaw.cpp, sendUnsigned7Bit()
 *   uint8_t byte = static_cast<uint8_t>(lroundf(fabs(data) * INT8_MAX));
 *   sendTransaction(command, &byte, 1);   // comandos 0,1,4,5
 *
 * Os comandos Packet Serial 0, 1, 4 e 5 exigem um byte de dados entre 0 e 127
 * (RoboClaw User Manual, Rev. 6.0, pp. 48-49). No caminho de terminacao de voo,
 * o valor de failsafe (a module.yaml aceita ate 257) chega a esta funcao sem
 * passar por output_limit_calc(), entao fabs(data) pode passar de 1.0 e o byte
 * resultante passa de 127 (chega a 130 com failsafe = 257). A funcao nao aplica
 * constrain -- diferente de sendSigned16Bit(), que usa math::constrain().
 *
 * Propriedade verificada: o byte enviado nunca pode passar de 127.
 *
 * Compile com -DCORRECAO para aplicar o constrain(fabs(data),0,1) do patch.
 *
 *   esbmc harness_roboclaw_bug1.cpp --overflow-check
 *   esbmc harness_roboclaw_bug1.cpp -DCORRECAO --overflow-check
 *   cbmc  harness_roboclaw_bug1.cpp --signed-overflow-check
 *   cbmc  harness_roboclaw_bug1.cpp -DCORRECAO --signed-overflow-check
 */
#include <assert.h>

#define INT8_MAX_M 127

extern float nondet_float(void);        /* valor simbolico de 'data' */

static float fabsf_m(float x)                     { return x < 0.0f ? -x : x; }
static float constrain_m(float v, float lo, float hi) {
	return v < lo ? lo : (v > hi ? hi : v);
}
/* lroundf para valores >= 0: arredonda para o inteiro mais proximo */
static long  lroundf_m(float x)                   { return (long)(x + 0.5f); }

int main(void)
{
	float data = nondet_float();

	/* Faixa que o caminho de failsafe pode entregar: fabs(data) de 0 a 130/127,
	   pois o failsafe maximo (257) pela module.yaml corresponde ao byte 130. */
	if (!(data >= 0.0f && data <= 130.0f / 127.0f)) {
		return 0;               /* fora da faixa real: nao analisado */
	}

#ifdef CORRECAO
	unsigned char byte = (unsigned char)
		lroundf_m(constrain_m(fabsf_m(data), 0.0f, 1.0f) * INT8_MAX_M);
#else
	unsigned char byte = (unsigned char)
		lroundf_m(fabsf_m(data) * INT8_MAX_M);
#endif

	/* Comandos 0, 1, 4 e 5 exigem um byte entre 0 e 127. */
	assert(byte <= INT8_MAX_M);
	return 0;
}
