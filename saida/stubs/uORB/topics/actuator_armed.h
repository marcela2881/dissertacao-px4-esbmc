/* GERADO por saida/scripts/msg2stub.py a partir de msg/ActuatorArmed.msg (PX4 8be2590). */
#pragma once
#include <cstdint>
struct actuator_armed_s {
	uint64_t timestamp;
	bool armed;
	bool prearmed;
	bool ready_to_arm;
	bool lockdown;
	bool kill;
	bool termination;
	bool in_esc_calibration_mode;

};
