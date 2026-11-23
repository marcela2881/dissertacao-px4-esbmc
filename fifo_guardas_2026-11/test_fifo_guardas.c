/* test_fifo_guardas.c
 *
 * Modelo do caminho de leitura do FIFO do acelerometro BMI088 (PX4, commit 8be2590c15):
 *   src/drivers/imu/bosch/bmi088/BMI088_Accelerometer.cpp
 * As tres guardas que a dissertacao cita para refutar o candidato "count = 9088":
 *   G1  Run(), linha 218 : if (fifo_byte_counter >= FIFO::SIZE) { FIFOReset(); }
 *   G2  Run(), linha 235 : if (samples > FIFO_MAX_SAMPLES) { FIFOReset(); samples = 0; }
 *   G3  FIFORead(), linha 475 : transfer_size = math::min(samples * sizeof(FIFO::DATA) + 4, FIFO::SIZE);
 *
 * Constantes (todas do PX4 no commit analisado):
 *   FIFO::SIZE = 1024                      (Bosch_BMI088_Accelerometer_Registers.hpp, linha 152)
 *   sizeof(FIFO::DATA) = 7                 (static_assert, mesmo arquivo, linha 170)
 *   FIFO_MAX_SAMPLES = min(1024/7, 32)=32  (BMI088_Accelerometer.hpp, linha 61; o vetor x de sensor_accel_fifo tem 32)
 *   sizeof(FIFOTransferBuffer) = 4 + 32*7 = 228   (static_assert, BMI088_Accelerometer.hpp, linha 72)
 *   fifo_byte_counter e de 14 bits: combine(FIFO_LENGTH_1 & 0x3F, FIFO_LENGTH_0), maximo 0x3FFF.
 *
 * Propriedades (uma por execucao, escolhida com -DPROP=1 ou -DPROP=2):
 *   PROP=1  quando ha leitura (samples >= 1), fifo_byte_counter < FIFO::SIZE      (o candidato refutado)
 *   PROP=2  quando ha leitura, transfer_size <= sizeof(FIFOTransferBuffer)        (o SPI nao escreve alem do buffer)
 *
 * Variantes: -DSEM_G1, -DSEM_G2, -DSEM_G3 removem cada guarda (para ver quais sao necessarias).
 * Escopo: so o caminho em que 'samples' vem da leitura do contador (Run(), samples == 0 na entrada).
 * Compilar para o CBMC com -DUSE_CBMC.
 */
#include <assert.h>
#include <stdint.h>
#include <stddef.h>

#ifdef USE_CBMC
#define ASSUME(c) __CPROVER_assume(c)
#else
/* __ESBMC_assume ja e declarada pelo proprio ESBMC (esbmc_intrinsics.h, argumento _Bool):
   nao declarar de novo, senao ha 'conflicting types' em arquivo .c */
#define ASSUME(c) __ESBMC_assume(c)
#endif
extern unsigned int nondet_uint(void);

#ifndef PROP
#define PROP 1
#endif

#define FIFO_SIZE         1024u
#define DATA_SIZE         7u
#define FIFO_MAX_SAMPLES  32u
#define BUF_BYTES         (4u + FIFO_MAX_SAMPLES * DATA_SIZE)   /* 228 */

static size_t min_sz(size_t a, size_t b) { return a < b ? a : b; }

int main(void)
{
	/* combine(FIFO_LENGTH_1 & 0x3F, FIFO_LENGTH_0): 14 bits, qualquer valor */
	unsigned int hi = nondet_uint();
	unsigned int lo = nondet_uint();
	ASSUME(hi <= 0x3Fu);
	ASSUME(lo <= 0xFFu);
	uint16_t fifo_byte_counter = (uint16_t)((hi << 8) | lo);

	/* _fifo_samples: ver linha 324 (limitado a FIFO_MAX_SAMPLES) */
	unsigned int fifo_samples_cfg = nondet_uint();
	ASSUME(fifo_samples_cfg <= FIFO_MAX_SAMPLES);

	uint8_t samples = 0;           /* Run(): uint8_t samples = 0; */

#ifndef SEM_G1
	if (fifo_byte_counter >= FIFO_SIZE) {
		/* G1: FIFOReset(); samples continua 0 */
	} else
#endif
	if ((fifo_byte_counter == 0) || (fifo_byte_counter == 0x8000)) {
		/* FIFO vazio */
	} else {
		samples = (uint8_t)(fifo_byte_counter / DATA_SIZE);   /* conversao size_t -> uint8_t, como no codigo */

		if (samples == fifo_samples_cfg + 1u) {               /* tolerancia de jitter */
			samples--;
		}

#ifndef SEM_G2
		if (samples > FIFO_MAX_SAMPLES) {
			/* G2: FIFOReset(); samples = 0; */
			samples = 0;
		}
#endif
	}

	if (samples >= 1) {                                       /* FIFORead(timestamp, samples) */
#ifndef SEM_G3
		size_t transfer_size = min_sz((size_t)samples * DATA_SIZE + 4u, FIFO_SIZE);   /* G3 */
#else
		size_t transfer_size = (size_t)samples * DATA_SIZE + 4u;
#endif
#if PROP == 1
		assert(fifo_byte_counter < FIFO_SIZE);
#else
		assert(transfer_size <= BUF_BYTES);
#endif
	}
	return 0;
}
