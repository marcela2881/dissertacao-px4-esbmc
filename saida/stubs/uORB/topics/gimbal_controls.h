/* GERADO por saida/scripts/msg2stub.py a partir de msg/GimbalControls.msg (PX4 8be2590). */
#pragma once
#include <cstdint>
struct gimbal_controls_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	float control[3];
	static constexpr uint8_t INDEX_ROLL = 0;
	static constexpr uint8_t INDEX_PITCH = 1;
	static constexpr uint8_t INDEX_YAW = 2;
};
