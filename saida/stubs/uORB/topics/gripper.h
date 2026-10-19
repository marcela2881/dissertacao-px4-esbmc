/* GERADO por saida/scripts/msg2stub.py a partir de msg/Gripper.msg (PX4 8be2590). */
#pragma once
#include <cstdint>
struct gripper_s {
	uint64_t timestamp;
	int8_t command;
	static constexpr int8_t COMMAND_GRAB = 0;
	static constexpr int8_t COMMAND_RELEASE = 1;
};
