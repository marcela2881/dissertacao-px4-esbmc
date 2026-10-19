/* GERADO por saida/scripts/msg2stub.py a partir de msg/ManualControlSetpoint.msg (PX4 8be2590). */
#pragma once
#include <cstdint>
struct manual_control_setpoint_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	bool valid;
	uint8_t data_source;
	float roll;
	float pitch;
	float yaw;
	float throttle;
	float flaps;
	float aux1;
	float aux2;
	float aux3;
	float aux4;
	float aux5;
	float aux6;
	bool sticks_moving;
	uint16_t buttons;
	static constexpr uint32_t MESSAGE_VERSION = 0;
	static constexpr uint8_t SOURCE_UNKNOWN = 0;
	static constexpr uint8_t SOURCE_RC = 1;
	static constexpr uint8_t SOURCE_MAVLINK_0 = 2;
	static constexpr uint8_t SOURCE_MAVLINK_1 = 3;
	static constexpr uint8_t SOURCE_MAVLINK_2 = 4;
	static constexpr uint8_t SOURCE_MAVLINK_3 = 5;
	static constexpr uint8_t SOURCE_MAVLINK_4 = 6;
	static constexpr uint8_t SOURCE_MAVLINK_5 = 7;
};
