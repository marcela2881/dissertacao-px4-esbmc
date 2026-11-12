/**
 * @file harness.cpp — batmon (smart battery): cell count vs voltage_cell_v[14]
 *
 * MODULO PX4: src/drivers/smart_battery/batmon/batmon.cpp e batmon.h
 * BASE: PX4 main c9b9e4e25e (13/10/2026)
 *
 * batmon.h:49    #define MAX_CELL_COUNT 16
 * batmon.h:90    float _cell_voltages[MAX_CELL_COUNT]
 * batmon.cpp:241 ret = _interface->read_word(BATT_SMBUS_CELL_COUNT, num_cells);
 * batmon.cpp:243 _cell_count = math::min((uint8_t)num_cells, (uint8_t)MAX_CELL_COUNT);
 * batmon.cpp:143 for (int i = 0; i < _cell_count; i++) {
 * batmon.cpp:144     new_report.voltage_cell_v[i] = _cell_voltages[i];
 *
 * battery_status_s::voltage_cell_v e float32[14] (msg/versioned/BatteryStatus.msg:38).
 * A struct abaixo e a gerada pelo PX4 (Tools/msg, uORB) para BatteryStatus.msg,
 * na mesma ordem de campos.
 *
 * O numero de celulas vem do registrador SMBus 0x40 da bateria (entrada externa,
 * livre). As tensoes de celula tambem vem da bateria.
 * Compilar com -DCORRECAO limita _cell_count ao tamanho de voltage_cell_v, como
 * o PR #28606 fez no driver UAVCAN (cbat_sub_cb).
 */

#include <assert.h>
#include <stdint.h>
#include <string.h>

extern uint16_t nondet_uint16();

#define MAX_CELL_COUNT 16

struct battery_status_s {           // gerada de msg/versioned/BatteryStatus.msg
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

static uint8_t _cell_count;
static float _cell_voltages[MAX_CELL_COUNT];

static uint8_t min_u8(uint8_t a, uint8_t b) { return a < b ? a : b; }

// batmon.cpp:240-243 (init): numero de celulas lido da bateria
static void read_cell_count()
{
	uint16_t num_cells = nondet_uint16();      // read_word(BATT_SMBUS_CELL_COUNT)
#ifdef CORRECAO
	const uint8_t cap = sizeof(((battery_status_s *)0)->voltage_cell_v) / sizeof(float);
	_cell_count = min_u8(min_u8((uint8_t)num_cells, (uint8_t)MAX_CELL_COUNT), cap);
#else
	_cell_count = min_u8((uint8_t)num_cells, (uint8_t)MAX_CELL_COUNT);
#endif
}

// batmon.cpp:277-281 (get_cell_voltages): tensoes lidas da bateria
static void get_cell_voltages()
{
	for (int i = 0; i < _cell_count; i++) {
		_cell_voltages[i] = (float)nondet_uint16() * 0.001f;
	}
}

// batmon.cpp:130-145 (RunImpl): copia para o relatorio publicado
static void run_impl(battery_status_s &new_report)
{
	get_cell_voltages();

	for (int i = 0; i < _cell_count; i++) {
		assert(i < 14);   // indice valido de voltage_cell_v
		new_report.voltage_cell_v[i] = _cell_voltages[i];
	}
}

int main()
{
	read_cell_count();
	battery_status_s new_report = {};   // batmon.cpp:130
	run_impl(new_report);
	return 0;
}
