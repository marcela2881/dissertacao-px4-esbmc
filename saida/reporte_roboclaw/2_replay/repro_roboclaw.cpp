/*
 * Reproduction of the two Roboclaw driver bugs on a PC, with an emulated UART.
 *
 * The ORIGINAL src/drivers/roboclaw/Roboclaw.cpp is compiled unchanged; only the
 * POSIX serial calls (write/read/select/open/termios) are replaced by the emulation
 * below. Each test has the same three steps:
 *   1. input preparation
 *   2. execution of a Roboclaw function
 *   3. an assert that is violated
 *
 *   g++ -std=c++17 -O0 -I <stubs> -I <PX4>/src/drivers/roboclaw repro_roboclaw.cpp -o repro
 *   ./repro r1    # speed byte above 127
 *   ./repro r3    # write() error not detected
 */
#include <cassert>
#include <cerrno>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <sys/types.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

/* ---- emulated UART ---- */
static bool g_write_fails = false;     /* write() returns -1 (e.g. EIO, adapter unplugged) */
static bool g_last_write_failed = false;

extern "C" {
ssize_t write(int, const void *buf, size_t n)
{
	const uint8_t *p = static_cast<const uint8_t *>(buf);

	/* Packet Serial commands 0, 1, 4, 5 (Drive Forward/Backwards M1/M2):
	 * [address, command, data, CRC_H, CRC_L]. RoboClaw User Manual Rev. 6.0,
	 * "Commands 0 - 7 Compatibility Commands", pp. 48-49: data range is 0 - 127. */
	if (n == 5 && (p[1] == 0 || p[1] == 1 || p[1] == 4 || p[1] == 5)) {
		printf("  sent: command %u, data byte %u\n", p[1], p[2]);
		assert(p[2] <= 127 && "drive command data byte must be 0..127");
	}

	g_last_write_failed = g_write_fails;

	if (g_write_fails) { errno = EIO; return -1; }

	return (ssize_t)n;
}

int select(int, fd_set *, fd_set *, fd_set *, struct timeval *)
{
	/* After a failed write() nothing was sent, so the driver must not wait for a reply. */
	assert(!g_last_write_failed && "select() called after write() returned -1");
	return 0;                          /* device silent: timeout */
}

ssize_t read(int, void *buf, size_t n) { if (n) { ((uint8_t *)buf)[0] = 0xFF; return 1; } return 0; }
int open(const char *, int, ...) { return 3; }
int close(int) { return 0; }
int tcgetattr(int, struct termios *) { return 0; }
int tcsetattr(int, int, const struct termios *) { return 0; }
int cfsetispeed(struct termios *, speed_t) { return 0; }
int cfsetospeed(struct termios *, speed_t) { return 0; }
}

#include "Roboclaw.hpp"
#include "Roboclaw.cpp"     /* ORIGINAL PX4 source */

int32_t g_param_values[px4::params::PARAM_COUNT];

int main(int argc, char **argv)
{
	setvbuf(stdout, nullptr, _IONBF, 0);   /* keep the output visible when assert() aborts */
	const char *test = argc > 1 ? argv[1] : "r1";
	g_param_values[px4::params::RBCLW_ADDRESS] = 128;      /* default 0x80 */
	g_param_values[px4::params::RBCLW_COUNTS_REV] = 1200;  /* default */
	Roboclaw dev("/dev/ttyS2", "RBCLW_BAUD");

	if (strcmp(test, "r1") == 0) {
		/* 1. Input preparation: RBCLW_FAIL1 = 257 (allowed by module.yaml,
		 *    failsafe: { min: 0, max: 257 }). In flight termination,
		 *    MixingOutput::limitAndUpdateOutputs() passes the failsafe value
		 *    unchanged: outputs[0] = (float)_failsafe_value[0] = 257. */
		float outputs[2] = {257.f, 127.f};
		printf("R1: outputs[0] = %.0f (RBCLW_FAIL1 = 257, flight termination)\n", outputs[0]);

		/* 2. Execution: Roboclaw::updateOutputs(), called by the mixer. */
		dev.updateOutputs(outputs, 2, 1);

		/* 3. The assert in the emulated write() is violated: data byte 130 > 127. */
		printf("R1: not reproduced (data byte within 0..127)\n");
		return 1;
	}

	if (strcmp(test, "r3") == 0) {
		/* 1. Input preparation: the serial port fails, write() returns -1. */
		g_write_fails = true;
		printf("R3: write() returns -1 (errno = EIO)\n");

		/* 2. Execution: Roboclaw::readEncoder(), called every cycle by Run().
		 *    writeCommand() stores -1 in a size_t, so "bytes_written < 2" is false
		 *    and it returns OK; readResponse() then calls select(). */
		dev.readEncoder();

		/* 3. The assert in the emulated select() is violated. */
		printf("R3: not reproduced (the write error was detected)\n");
		return 1;
	}

	printf("use: %s r1|r3\n", argv[0]);
	return 2;
}
