/**
 * @file replay.cpp — batmon: bateria de 16 celulas (registradores 0x3F..0x30)
 *
 * Reproduz batmon.cpp:130-145 e :190-196 (RunImpl) com a struct gerada de
 * BatteryStatus.msg. A bateria informa 16 celulas (0x0010) e tensoes de
 * 3.700 V .. 3.715 V. full_charge_capacity_wh nao e preenchido pelo batmon,
 * entao deveria ser publicado como 0.
 *
 *   ./replay 16    (ou 15)
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MAX_CELL_COUNT 16

struct battery_status_s {
	uint64_t timestamp;
	float voltage_v;
	float current_a;
	float current_average_a;
	float discharged_mah;
	float remaining;
	float scale;
	float time_remaining_s;
	float temperature;
	float voltage_cell_v[14];
	float max_cell_voltage_delta;
	float full_charge_capacity_wh;
	float remaining_capacity_wh;
	float nominal_voltage;
	float internal_resistance_estimate;
	float ocv_estimate;
	float ocv_estimate_filtered;
	float volt_based_soc_estimate;
	float voltage_prediction;
	float prediction_error;
	float estimation_covariance_norm;
	uint16_t capacity;
	uint16_t cycle_count;
	uint16_t average_time_to_empty;
	uint16_t manufacture_date;
	uint16_t state_of_health;
	uint16_t max_error;
	uint16_t interface_error;
	uint16_t faults;
	uint16_t over_discharge_count;
	bool connected;
	uint8_t cell_count;
	uint8_t source;
	uint8_t priority;
	uint8_t id;
	bool is_powering_off;
	bool is_required;
	uint8_t warning;
	uint8_t _padding0[2];
};

template <typename T> static T min_px4(T a, T b) { return a < b ? a : b; }

int main(int argc, char **argv)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	uint16_t num_cells = (uint16_t)((argc > 1) ? atoi(argv[1]) : 16);   // registrador 0x40

	// batmon.cpp:243
	uint8_t _cell_count = min_px4((uint8_t)num_cells, (uint8_t)MAX_CELL_COUNT);

	// batmon.cpp:277-281
	float _cell_voltages[MAX_CELL_COUNT] = {};
	for (int i = 0; i < _cell_count; i++) {
		_cell_voltages[i] = (float)(3700 + i) * 0.001f;
	}
	float _max_cell_voltage_delta = 0.0075f;

	// batmon.cpp:130-145
	battery_status_s new_report = {};
	for (int i = 0; i < _cell_count; i++) {
		new_report.voltage_cell_v[i] = _cell_voltages[i];
	}

	// batmon.cpp:190-196 (so se a leitura SMBus deu certo)
	new_report.max_cell_voltage_delta = _max_cell_voltage_delta;
	new_report.cell_count = _cell_count;

	printf("cell_count=%u (voltage_cell_v tem 14 posicoes)\n", new_report.cell_count);
	printf("max_cell_voltage_delta=%.4f (rescrito depois do laco)\n", new_report.max_cell_voltage_delta);
	printf("full_charge_capacity_wh=%.4f (batmon nao preenche; esperado 0)\n", new_report.full_charge_capacity_wh);
	return 0;
}
