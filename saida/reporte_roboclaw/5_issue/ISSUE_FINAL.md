### Title

Roboclaw: failsafe values above 254 produce speed bytes above 127, and write() errors are not detected

### Describe the bug

I found two bugs in the Roboclaw driver (`src/drivers/roboclaw/Roboclaw.cpp`) and confirmed both on a PC, by compiling and running the original `Roboclaw.cpp` with an emulated UART.

#### 1. Speed byte can be greater than 127

The driver uses Packet Serial commands 0, 1, 4 and 5 (Drive Forward/Backwards M1/M2). According to the RoboClaw User Manual (Rev. 6.0, "Commands 0 - 7 Compatibility Commands", pp. 48–49), the data byte for these commands must be between 0 and 127.

The driver calculates this byte in two steps, without limiting the value:

- `updateOutputs()`: `(outputs[i] - 127) / 127` (lines 161–162)
- `sendUnsigned7Bit()`: `lroundf(fabs(data) * INT8_MAX)` (line 291)

For outputs between 0 and 254 the byte stays between 0 and 127. However, `module.yaml` allows failsafe values up to 257 (`failsafe: { min: 0, max: 257 }`), and during flight termination the mixer passes the failsafe value to the driver unchanged (`mixer_module.cpp`, lines 497–501, `actualFailsafeValue()`):

| RBCLW_FAILn | byte sent |
|---|---|
| 254 | 127 |
| 255 | **128** |
| 256 | **129** |
| 257 | **130** |

I have not tested on hardware what the RoboClaw does with these values.

#### 2. write() errors are not detected

In `writeCommand()` (lines 380–385) and `writeCommandWithPayload()` (lines 332–338), the return value of `write()` is stored in a `size_t`:

```cpp
size_t bytes_written = write(_uart_fd, buffer, 2);
if (bytes_written < 2) { ... return ERROR; }
return OK;
```

When `write()` fails it returns -1, which becomes a very large unsigned number, so `bytes_written < 2` is false and the function returns `OK`. The caller then waits for a reply that cannot arrive (`select()` in `readResponse()` / `readAcknowledgement()`).

### Other protections checked

I checked whether anything else in the code path prevents these bugs:

1. **Speed byte.** On the path `MixingOutput::limitAndUpdateOutputs()` → `actualFailsafeValue()` → `Roboclaw::updateOutputs()` → `setMotorSpeed()` → `sendUnsigned7Bit()` → `writeCommandWithPayload()` → `write()`, no function limits the value. The mixer limits outputs to [min, max] only in the normal path (`output_limit_calc()`), not in the termination path. `sendSigned16Bit()` uses `math::constrain()`, but `sendUnsigned7Bit()` does not. The failsafe value 255–257 is inside the range declared in `module.yaml`, so the parameter system accepts it. Flight termination can be reached while armed through `MAV_CMD_DO_FLIGHTTERMINATION` or through failsafe actions set to "Terminate" (for example `NAV_RCL_ACT`, `NAV_DLL_ACT`); `CBRK_FLIGHTTERM` only disables termination triggered by the failure detector.
2. **write() error.** The port is opened in blocking mode, and nothing between `write()` and the size check handles the error. The failure is noticed later only because `select()` times out (11 ms per transaction), so the driver logs "ACK timeout" / "Select timeout" instead of the write error, and the message "Only wrote ... bytes" is never printed.

### To Reproduce

Unit tests (gtest) that only need to be compiled and linked with the Roboclaw driver. `Roboclaw.cpp` is not changed; the UART is emulated at link time (`-Wl,--wrap=write -Wl,--wrap=select`), so the driver's `write()` and `select()` calls on its serial file descriptor reach the test. Each test has three steps: input preparation, execution of a Roboclaw function, and a check that fails while the bug is present.

1. Add `RoboclawTest.cpp` (attached / <LINK-DO-ARTEFATO>) to `src/drivers/roboclaw/`.
2. Add to `src/drivers/roboclaw/CMakeLists.txt`:

```cmake
px4_add_functional_gtest(SRC RoboclawTest.cpp LINKLIBS drivers__roboclaw)
if(TARGET functional-Roboclaw)
	target_link_options(functional-Roboclaw PRIVATE -Wl,--wrap=write -Wl,--wrap=select)
endif()
```

3. Enable the driver in the SITL test build (`CONFIG_DRIVERS_ROBOCLAW=y` in `boards/px4/sitl/test.px4board`) and run `make tests TESTFILTER=Roboclaw`.

The two tests:

```cpp
// Bug 1
TEST_F(RoboclawTest, FailsafeAbove254KeepsDriveDataByteWithin127)
{
	// 1. Input preparation: RBCLW_FAIL1 = 257, passed unchanged by the mixer in flight termination
	float outputs[2] = {257.f, 127.f};
	// 2. Execution
	_roboclaw.updateOutputs(outputs, 2, 1);
	// 3. Check: data byte of commands 0, 1, 4, 5 must be 0..127
	for (const auto &packet : g_packets) {
		if (isDriveCommand(packet[1])) { EXPECT_LE(static_cast<int>(packet[2]), 127); }
	}
}

// Bug 2
TEST_F(RoboclawTest, WriteErrorIsDetected)
{
	// 1. Input preparation: write() returns -1 (errno = EIO)
	g_write_fails = true;
	// 2. Execution
	_roboclaw.readEncoder();
	// 3. Check: no reply is awaited after a failed write()
	EXPECT_EQ(g_select_calls_after_failed_write, 0);
}
```

Result on the current code:

```
<SAIDA-DO-make-tests>
Expected: (static_cast<int>(packet[2])) <= (127), actual: 130 vs 127
[  FAILED  ] RoboclawTest.FailsafeAbove254KeepsDriveDataByteWithin127
Expected equality of these values:
  g_select_calls_after_failed_write
    Which is: 1
  0
[  FAILED  ] RoboclawTest.WriteErrorIsDetected
```

With the fix below applied, both tests pass.

On a vehicle: set `RBCLW_FAIL1` to 255, 256 or 257 and trigger flight termination.

### Expected behavior

1. The speed byte should never be greater than 127. Possible fixes:
   - limit the value in the driver, for example with `math::constrain(fabsf(data), 0.f, 1.f)` in `sendUnsigned7Bit()`; or
   - change the failsafe limit in `module.yaml` from 257 to 254 (the same as `max`).
2. If `write()` fails, the function should return `ERROR`, for example by storing the return value in a `ssize_t` and checking it before comparing with the packet size.

### Software Version

PX4-Autopilot commit `8be2590c1529517ccd54fbc09ae138402517baf4`. The same code is still on `main` (checked on 2026-10-19).

### Flight controller / Vehicle type

Rover with Roboclaw. No flight log is available; the bugs were confirmed on a PC with an emulated UART.
