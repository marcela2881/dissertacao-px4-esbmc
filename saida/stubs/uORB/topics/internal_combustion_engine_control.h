/* GERADO por saida/scripts/msg2stub.py a partir de msg/InternalCombustionEngineControl.msg (PX4 8be2590). */
#pragma once
#include <cstdint>
struct internal_combustion_engine_control_s {
	uint64_t timestamp;
	bool ignition_on;
	float throttle_control;
	float choke_control;
	float starter_engine_control;
	uint8_t user_request;

};
