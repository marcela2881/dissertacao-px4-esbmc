/* GERADO por saida/scripts/msg2stub.py a partir de msg/ControlAllocatorStatus.msg (PX4 8be2590). */
#pragma once
#include <cstdint>
struct control_allocator_status_s {
	uint64_t timestamp;
	bool torque_setpoint_achieved;
	float unallocated_torque[3];
	bool thrust_setpoint_achieved;
	float unallocated_thrust[3];
	int8_t actuator_saturation[16];
	uint16_t handled_motor_failure_mask;
	uint16_t motor_stop_mask;
	bool actuator_group_preflight_check_active;
	static constexpr int8_t ACTUATOR_SATURATION_OK = 0;
	static constexpr int8_t ACTUATOR_SATURATION_UPPER_DYN = 1;
	static constexpr int8_t ACTUATOR_SATURATION_UPPER = 2;
	static constexpr int8_t ACTUATOR_SATURATION_LOWER_DYN = -1;
	static constexpr int8_t ACTUATOR_SATURATION_LOWER = -2;
};
