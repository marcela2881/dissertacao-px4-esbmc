/*
 * REPLAY (teste unitario) -- IMU BMI088 (acelerometro), contraexemplos da fase 1:
 * temperatura (Issue #26865) e contador da FIFO (PR #27575).
 *
 * Executa as funcoes ORIGINAIS do PX4: UpdateTemperature() e FIFORead() sao
 * extraidas de src/drivers/imu/bosch/bmi088/BMI088_Accelerometer.cpp pelo
 * rodar_replay.sh (gen/bmi088_*.inc), sem alteracao; os registradores vem do
 * Bosch_BMI088_Accelerometer_Registers.hpp original. So o ambiente da classe e
 * simulado (transferencia SPI, contadores perf, publicacao). Compilado com
 * -fsanitize=address,undefined.
 *
 *   ./replay_bmi088 temperatura : registradores do contraexemplo (MSB=63, LSB=95)
 *   ./replay_bmi088 fifo        : contador do contraexemplo (9088 bytes) na leitura da FIFO
 *   ./replay_bmi088 fifo_aleatorio : 200.000 leituras com contador e bytes da FIFO
 *                                 aleatorios (SPI com ruido), amostras 1..32
 */
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>

typedef uint64_t hrt_abstime;
#define PX4_OK 0
#define PX4_ISFINITE(x) std::isfinite(x)
#define PX4_DEBUG(...) ((void)0)
namespace math { template<class T> constexpr T min(T a, T b) { return a < b ? a : b; } }

struct sensor_accel_fifo_s {              /* msg/SensorAccelFifo.msg */
	uint64_t timestamp, timestamp_sample;
	uint32_t device_id;
	float dt, scale;
	uint8_t samples;
	int16_t x[32], y[32], z[32];
};

#include "Bosch_BMI088_Accelerometer_Registers.hpp"   /* ORIGINAL */

static uint8_t g_spi[1024 + 8];           /* bytes que o "sensor" devolve na transferencia */
static float   g_temperatura = NAN;
static int     g_amostras_publicadas = -1;

namespace Bosch::BMI088::Accelerometer
{
static constexpr int16_t combine(uint8_t msb, uint8_t lsb) { return (msb << 8u) | lsb; }   /* BMI088.hpp:41 */

class BMI088_Accelerometer
{
public:
	static constexpr uint32_t RATE{1600};
	static constexpr float FIFO_SAMPLE_DT{1e6f / RATE};
	static constexpr int32_t FIFO_MAX_SAMPLES{math::min(FIFO::SIZE / sizeof(FIFO::DATA), sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]))};

	struct FIFOTransferBuffer {           /* BMI088_Accelerometer.hpp, identico */
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_LENGTH_0) | DIR_READ};
		uint8_t dummy{0};
		uint8_t FIFO_LENGTH_0{0};
		uint8_t FIFO_LENGTH_1{0};
		FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	};
	static_assert(sizeof(FIFOTransferBuffer) == (4 + FIFO_MAX_SAMPLES *sizeof(FIFO::DATA)));

	int transfer(uint8_t *send, uint8_t *recv, size_t len) { memcpy(recv, g_spi, len); return PX4_OK; }

	int _bad_transfer_perf{0}, _bad_register_perf{0}, _fifo_empty_perf{0}, _fifo_overflow_perf{0};
	struct {
		void set_temperature(float t) { g_temperatura = t; }
		void set_error_count(uint64_t) {}
		void updateFIFO(sensor_accel_fifo_s &a) { g_amostras_publicadas = a.samples; }
	} _px4_accel;

	bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	void UpdateTemperature();
};

#define perf_count(x) ((x)++)
#define perf_event_count(x) ((uint64_t)(x))

#include "gen/bmi088_FIFORead.inc"            /* corpo ORIGINAL */
#include "gen/bmi088_UpdateTemperature.inc"   /* corpo ORIGINAL */
}

using Bosch::BMI088::Accelerometer::BMI088_Accelerometer;

int main(int argc, char **argv)
{
	const char *modo = argc > 1 ? argv[1] : "temperatura";
	BMI088_Accelerometer imu;

	if (strcmp(modo, "temperatura") == 0) {
		printf("=== BMI088 temperatura: TEMP_MSB = 63, TEMP_LSB = 95 (contraexemplo) ===\n");
		memset(g_spi, 0, sizeof(g_spi));
		g_spi[2] = 63; g_spi[3] = 95;
		imu.UpdateTemperature();
		printf("temperatura repassada ao PX4: %.2f C (faixa do datasheet: -40 a 85 C) -> %s\n",
		       g_temperatura, (g_temperatura > 85.f || g_temperatura < -40.f) ? "FORA da faixa, sem limitacao no driver" : "dentro");
		return 0;
	}

	if (strcmp(modo, "fifo") == 0) {
		printf("=== BMI088 FIFO: contador do contraexemplo = 9088 bytes (FIFO de 1024) ===\n");
		memset(g_spi, 0, sizeof(g_spi));
		g_spi[2] = 9088 & 0xFF; g_spi[3] = (9088 >> 8) & 0x3F;
		bool ok = imu.FIFORead(0, BMI088_Accelerometer::FIFO_MAX_SAMPLES);
		printf("FIFORead devolveu %s; contador de overflow = %d\n", ok ? "true" : "false", imu._fifo_overflow_perf);
		printf("=> a guarda 'fifo_byte_counter >= FIFO::SIZE' descarta a leitura: nenhum acesso com 9088 bytes\n");
		return 0;
	}

	printf("=== BMI088 FIFO aleatoria: 200.000 leituras (contador 0..0x3FFF, bytes aleatorios) ===\n");
	srand(4242);
	unsigned maior = 0;

	for (int i = 0; i < 200000; i++) {
		for (size_t k = 0; k < sizeof(g_spi); k++) { g_spi[k] = rand() & 0xFF; }

		g_spi[3] &= 0x3F;
		uint8_t amostras = 1 + rand() % BMI088_Accelerometer::FIFO_MAX_SAMPLES;
		g_amostras_publicadas = -1;
		imu.FIFORead(0, amostras);

		if (g_amostras_publicadas > (int)maior) { maior = g_amostras_publicadas; }
	}

	printf("maior numero de amostras publicadas: %u (limite %d); nenhum erro de memoria\n", maior, (int)BMI088_Accelerometer::FIFO_MAX_SAMPLES);
	return 0;
}
