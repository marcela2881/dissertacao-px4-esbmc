/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Unit tests for the Roboclaw driver with an emulated UART.
 *
 * Roboclaw.cpp is linked unchanged (LINKLIBS drivers__roboclaw). The UART is
 * emulated at link time: the test is linked with -Wl,--wrap=write and
 * -Wl,--wrap=select, so the driver's calls to write() and select() on its
 * serial file descriptor reach the functions below. All other calls go to the
 * real functions.
 *
 * The tests call the public methods directly, without starting the module, so
 * initializeUART() does not run and the driver's file descriptor keeps its
 * initial value (_uart_fd{0}).
 *
 * Each test has three steps: input preparation, execution of a Roboclaw
 * function, and the check that fails while the bug is present.
 */

#include <gtest/gtest.h>

#include <cerrno>
#include <cstdint>
#include <vector>
#include <sys/select.h>
#include <unistd.h>

#include "Roboclaw.hpp"

namespace
{
constexpr int kUartFd = 0;               // Roboclaw::_uart_fd before initializeUART()

bool g_emulate = false;                  // only intercept calls made during a test
bool g_write_fails = false;              // write() returns -1 (errno = EIO)
bool g_last_write_failed = false;
int g_select_calls_after_failed_write = 0;
std::vector<std::vector<uint8_t>> g_packets;
}

extern "C" {
ssize_t __real_write(int fd, const void *buf, size_t count);
int __real_select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout);

ssize_t __wrap_write(int fd, const void *buf, size_t count)
{
	if (!g_emulate || fd != kUartFd) {
		return __real_write(fd, buf, count);
	}

	const uint8_t *bytes = static_cast<const uint8_t *>(buf);
	g_packets.emplace_back(bytes, bytes + count);
	g_last_write_failed = g_write_fails;

	if (g_write_fails) {
		errno = EIO;
		return -1;
	}

	return static_cast<ssize_t>(count);
}

int __wrap_select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout)
{
	if (!g_emulate || nfds != kUartFd + 1) {
		return __real_select(nfds, readfds, writefds, exceptfds, timeout);
	}

	if (g_last_write_failed) {
		g_select_calls_after_failed_write++;
	}

	return 0; // the Roboclaw does not answer: timeout
}
}

class RoboclawTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		g_emulate = true;
		g_write_fails = false;
		g_last_write_failed = false;
		g_select_calls_after_failed_write = 0;
		g_packets.clear();
	}

	void TearDown() override
	{
		g_emulate = false;
	}

	// Packet Serial drive commands: Drive Forward/Backwards M1/M2
	static bool isDriveCommand(uint8_t command)
	{
		return command == 0 || command == 1 || command == 4 || command == 5;
	}

	Roboclaw _roboclaw{"/dev/null", "RBCLW_BAUD"};
};

// Bug 1: RoboClaw User Manual Rev. 6.0, "Commands 0 - 7 Compatibility Commands",
// pp. 48-49: the data byte of commands 0, 1, 4 and 5 must be 0..127.
TEST_F(RoboclawTest, FailsafeAbove254KeepsDriveDataByteWithin127)
{
	// 1. Input preparation: RBCLW_FAIL1 = 257 is allowed by module.yaml
	//    (failsafe: { min: 0, max: 257 }). During flight termination
	//    MixingOutput::limitAndUpdateOutputs() passes it unchanged.
	float outputs[2] = {257.f, 127.f};

	// 2. Execution
	_roboclaw.updateOutputs(outputs, 2, 1);

	// 3. Check: every drive command sent has a data byte of at most 127
	//    (with the bug, the first packet carries 130).
	ASSERT_FALSE(g_packets.empty());

	for (const auto &packet : g_packets) {
		ASSERT_GE(packet.size(), 3u);

		if (isDriveCommand(packet[1])) {
			EXPECT_LE(static_cast<int>(packet[2]), 127) << "command " << static_cast<int>(packet[1]);
		}
	}
}

// Bug 2: write() returns -1 on error (POSIX). A failed write must be reported
// as an error, without waiting for a reply that cannot arrive.
TEST_F(RoboclawTest, WriteErrorIsDetected)
{
	// 1. Input preparation: the serial port fails
	g_write_fails = true;

	// 2. Execution
	_roboclaw.readEncoder();

	// 3. Check: the driver did not wait for a reply after the failed write()
	//    (with the bug, writeCommand() returns OK and readResponse() calls select()).
	ASSERT_FALSE(g_packets.empty());
	EXPECT_EQ(g_select_calls_after_failed_write, 0);
}
