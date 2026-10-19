/* STUB de uORB/topics/esc_status.h: só a constante usada (msg/EscStatus.msg, commit fixado). */
#pragma once
#include <cstdint>
struct esc_status_s {
	static constexpr uint8_t CONNECTED_ESC_MAX = 12;   /* "Motor1-Motor12" */
};
