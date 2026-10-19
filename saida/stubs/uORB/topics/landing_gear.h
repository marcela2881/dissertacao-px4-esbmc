/* GERADO por saida/scripts/msg2stub.py a partir de msg/LandingGear.msg (PX4 8be2590). */
#pragma once
#include <cstdint>
struct landing_gear_s {
	uint64_t timestamp;
	int8_t landing_gear;
	static constexpr int8_t GEAR_UP = 1;
	static constexpr int8_t GEAR_DOWN = -1;
	static constexpr int8_t GEAR_KEEP = 0;
};
