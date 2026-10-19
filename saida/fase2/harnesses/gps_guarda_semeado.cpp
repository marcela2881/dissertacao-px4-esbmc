/*
 * Fase 2 -> fechamento do caso do PATCH DO GPS (PR #27576), apontado pelo orientador.
 *
 * Problema (fase 2): o CBMC deu 0/7 no harness CORRIGIDO e 0/7 no NAO corrigido,
 * porque os dois usavam a pre-condicao len < 200, que ja exclui a falha. A
 * comparacao nao isolava o efeito da guarda; e nao havia log do ESBMC.
 *
 * Aqui: UM harness, a MESMA pre-condicao semeada (len >= GPS_DUMP_DATA_SIZE - 10,
 * a mesma de test_gps_cbmc_fair.cpp), e a guarda ligada/desligada por -DCOM_GUARDA:
 *   -DCOM_GUARDA=0 : dumpGpsData como no harness da fase 2 (test_gps_cbmc_fair.cpp)
 *   -DCOM_GUARDA=1 : dumpGpsData_PATCHED como no harness da fase 2 (test_gps_patch.cpp)
 * Esperado: sem guarda FAILED; com guarda SUCCESSFUL (ESBMC e CBMC).
 * Laco: comeca com len >= 190 e input_len <= 300 -> no maximo 4 iteracoes; --unwind 5.
 */
#include <assert.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

extern "C" {
unsigned int nondet_uint();
uint8_t nondet_uint8_t();
void __VERIFIER_assume(int);
}

#ifndef COM_GUARDA
#define COM_GUARDA 0
#endif

#define GPS_DUMP_DATA_SIZE 200

struct gps_dump_s {
	uint8_t data[GPS_DUMP_DATA_SIZE];
	uint8_t len;
	uint8_t instance;
};

#if COM_GUARDA == 0
/* identico a test_gps_cbmc_fair.cpp (fase 2) */
void dumpGpsData(uint8_t *data, unsigned int len, gps_dump_s *dump_data)
{
	if (!dump_data) { return; }

	while (len > 0) {
		unsigned int write_len = len;

		if (write_len > GPS_DUMP_DATA_SIZE - dump_data->len) {
			write_len = GPS_DUMP_DATA_SIZE - dump_data->len;
		}

		memcpy(dump_data->data + dump_data->len, data, write_len);
		data += write_len;
		dump_data->len += write_len;
		len -= write_len;

		if (dump_data->len >= GPS_DUMP_DATA_SIZE) {
			dump_data->len = 0;
		}
	}
}
#else
/* identico a dumpGpsData_PATCHED de test_gps_patch.cpp (fase 2) */
void dumpGpsData(uint8_t *data, unsigned int len, gps_dump_s *dump_data)
{
	if (!dump_data) { return; }

	while (len > 0) {
		int available = (int)GPS_DUMP_DATA_SIZE - (int)dump_data->len;

		if (available <= 0) {
			dump_data->len = 0;
			break;
		}

		unsigned int write_len = len;

		if (write_len > (unsigned int)available) {
			write_len = (unsigned int)available;
		}

		memcpy(dump_data->data + dump_data->len, data, write_len);
		data += write_len;
		dump_data->len += write_len;
		len -= write_len;

		if (dump_data->len >= GPS_DUMP_DATA_SIZE) {
			dump_data->len = 0;
		}
	}
}
#endif

int main()
{
	unsigned int input_len = nondet_uint();
	__VERIFIER_assume(input_len > 0 && input_len <= 300);
	uint8_t input_data[300];
	gps_dump_s dump_buffer;
	dump_buffer.len = nondet_uint8_t();
	__VERIFIER_assume(dump_buffer.len >= GPS_DUMP_DATA_SIZE - 10);   /* janela semeada */
	dumpGpsData(input_data, input_len, &dump_buffer);
	assert(dump_buffer.len <= GPS_DUMP_DATA_SIZE);
	return 0;
}
