/* GERADO por saida/scripts/msg2stub.py a partir de msg/ActuatorOutputs.msg (PX4 8be2590). */
#pragma once
#include <cstdint>
struct actuator_outputs_s {
	uint64_t timestamp;
	uint32_t noutputs;
	float output[16];
	static constexpr uint8_t NUM_ACTUATOR_OUTPUTS = 16;
	static constexpr uint8_t NUM_ACTUATOR_OUTPUT_GROUPS = 4;
};
