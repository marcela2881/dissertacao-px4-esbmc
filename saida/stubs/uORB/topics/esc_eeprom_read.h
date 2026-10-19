/* STUB de uORB/topics/esc_eeprom_read.h: campos de msg/EscEepromRead.msg (commit fixado). */
#pragma once
#include <cstdint>
struct esc_eeprom_read_s {
	uint64_t timestamp;
	uint8_t  firmware;
	uint8_t  index;
	uint16_t length;
	uint8_t  data[48];
};
