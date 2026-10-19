/*
 * REPLAY (teste unitario) -- GPS uBlox, contraexemplo da fase 1 (Issue #26866).
 *
 * Executa a funcao GPS::dumpGpsData() ORIGINAL do PX4: o corpo e extraido de
 * src/drivers/gps/gps.cpp pelo rodar_replay.sh (gen/gps_dumpGpsData.inc), sem
 * alteracao. So o ambiente da classe e simulado (ponteiros de dump, modo,
 * instancia, publicacao uORB, tempo). gps_dump_s segue msg/GpsDump.msg (data[79]).
 * Compilado com -fsanitize=address,undefined.
 *
 *   ./replay_gps alcance  : parte do estado inicial do driver (len = 0) e faz
 *                           2 milhoes de chamadas com tamanhos 1..400, nos dois
 *                           sentidos; confere apos cada chamada que o
 *                           preenchimento fica abaixo de 79 (estado do
 *                           contraexemplo, len = 207, alcancavel ou nao?)
 *   ./replay_gps injetado : injeta o estado do contraexemplo (len = 207,
 *                           input_len = 10) na funcao de producao
 */
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cstdlib>

typedef uint64_t hrt_abstime;
static hrt_abstime hrt_absolute_time() { return 0; }

struct gps_dump_s {                       /* msg/GpsDump.msg */
	uint64_t timestamp;
	uint32_t device_id;
	uint8_t  instance;
	uint8_t  len;
	uint8_t  data[79];
};

enum class gps_dump_comm_mode_t : int32_t { Disabled = 0, Full, RTCM };   /* gps.cpp:106 */

static unsigned g_publicados = 0;

class GPS
{
public:
	gps_dump_s *_dump_to_device{nullptr};
	gps_dump_s *_dump_from_device{nullptr};
	gps_dump_comm_mode_t _dump_communication_mode{gps_dump_comm_mode_t::Full};
	int _instance{0};
	uint32_t get_device_id() { return 0; }
	struct { void publish(const gps_dump_s &) { g_publicados++; } } _dump_communication_pub;

	void dumpGpsData(const uint8_t *data, size_t len, gps_dump_comm_mode_t mode, bool msg_to_gps_device);
};

#include "gen/gps_dumpGpsData.inc"     /* corpo ORIGINAL de gps.cpp */

int main(int argc, char **argv)
{
	const char *modo = argc > 1 ? argv[1] : "alcance";
	static uint8_t entrada[400];
	GPS gps;
	gps._dump_to_device = new gps_dump_s();     /* alocado como no driver (zerado) */
	gps._dump_from_device = new gps_dump_s();

	if (strcmp(modo, "injetado") == 0) {
		printf("=== GPS injetado: estado do contraexemplo (len = 207, input_len = 10) ===\n");
		gps._dump_from_device->len = 207;
		fflush(stdout);
		gps.dumpGpsData(entrada, 10, gps_dump_comm_mode_t::Full, false);
		printf("dumpGpsData terminou sem erro de memoria (inesperado)\n");
		return 0;
	}

	printf("=== GPS alcance: estado inicial do driver (len = 0), 2.000.000 chamadas ===\n");
	srand(12345);
	unsigned max_len = 0;

	for (int i = 0; i < 2000000; i++) {
		size_t n = 1 + rand() % 400;
		bool para_dispositivo = rand() & 1;
		gps.dumpGpsData(entrada, n, gps_dump_comm_mode_t::Full, para_dispositivo);
		gps_dump_s *d = para_dispositivo ? gps._dump_to_device : gps._dump_from_device;

		if (d->len > max_len) { max_len = d->len; }

		if (d->len >= sizeof(d->data)) {
			printf("VIOLA: len = %u apos a chamada %d\n", d->len, i);
			return 1;
		}
	}

	printf("maior preenchimento observado: %u (capacidade 79); registros publicados: %u\n", max_len, g_publicados);
	printf("=> o estado do contraexemplo (len >= 79, ex.: 207) nao foi alcancado a partir do estado inicial\n");
	return 0;
}
