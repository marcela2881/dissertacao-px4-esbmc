/* GERADO por saida/scripts/msg2stub.py a partir de msg/ActuatorServosTrim.msg (PX4 8be2590). */
#pragma once
#include <cstdint>
struct actuator_servos_trim_s {
	uint64_t timestamp;
	float trim[15];
	static constexpr uint8_t NUM_CONTROLS = 15;
};
