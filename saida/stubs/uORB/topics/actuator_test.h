/* GERADO por saida/scripts/msg2stub.py a partir de msg/ActuatorTest.msg (PX4 8be2590). */
#pragma once
#include <cstdint>
struct actuator_test_s {
	uint64_t timestamp;
	uint8_t action;
	uint16_t function;
	float value;
	uint32_t timeout_ms;
	static constexpr uint8_t ACTION_RELEASE_CONTROL = 0;
	static constexpr uint8_t ACTION_DO_CONTROL = 1;
	static constexpr uint8_t FUNCTION_MOTOR1 = 101;
	static constexpr uint8_t MAX_NUM_MOTORS = 12;
	static constexpr uint8_t FUNCTION_SERVO1 = 201;
	static constexpr uint8_t MAX_NUM_SERVOS = 15;
	static constexpr uint8_t ORB_QUEUE_LENGTH = 16;
};
