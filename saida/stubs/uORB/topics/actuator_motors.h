/* GERADO por saida/scripts/msg2stub.py a partir de msg/ActuatorMotors.msg (PX4 8be2590). */
#pragma once
#include <cstdint>
struct actuator_motors_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	uint16_t reversible_flags;
	float control[12];
	static constexpr uint32_t MESSAGE_VERSION = 0;
	static constexpr uint8_t ACTUATOR_FUNCTION_MOTOR1 = 101;
	static constexpr uint8_t NUM_CONTROLS = 12;
};
