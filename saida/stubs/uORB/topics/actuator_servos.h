/* GERADO por saida/scripts/msg2stub.py a partir de msg/ActuatorServos.msg (PX4 8be2590). */
#pragma once
#include <cstdint>
struct actuator_servos_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	float control[15];
	static constexpr uint32_t MESSAGE_VERSION = 1;
	static constexpr uint8_t NUM_CONTROLS = 15;
};
