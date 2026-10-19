/*
 * STUB de drivers/drv_dshot.h: só os comandos DShot (valores copiados do
 * src/drivers/drv_dshot.h do commit fixado). O original inclui sys/ioctl.h e
 * drv_orb_dev.h e declara as funções de hardware (up_dshot_*), que o recorte
 * verificado (telemetria serial) não usa.
 */
#pragma once
#include <px4_platform_common/defines.h>   /* como no original (PX4_OK, PX4_ERROR) */
#include <stdint.h>
enum {
	DSHOT_CMD_MOTOR_STOP = 0,
	DSHOT_CMD_BEEP1 = 1,
	DSHOT_CMD_ESC_INFO = 6,
	DSHOT_CMD_SPIN_DIRECTION_1 = 7,
	DSHOT_CMD_SPIN_DIRECTION_2 = 8,
	DSHOT_CMD_3D_MODE_OFF = 9,
	DSHOT_CMD_3D_MODE_ON = 10,
	DSHOT_CMD_SAVE_SETTINGS = 12,
	DSHOT_EXTENDED_TELEMETRY_ENABLE = 13,
	DSHOT_CMD_ENTER_PROGRAMMING_MODE = 36,
	DSHOT_CMD_EXIT_PROGRAMMING_MODE = 37,
	DSHOT_CMD_MAX          = 47,
	DSHOT_CMD_MIN_THROTTLE = 48,
	DSHOT_CMD_MAX_THROTTLE = 2047
};
