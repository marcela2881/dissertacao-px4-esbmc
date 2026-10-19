# Title

Roboclaw: failsafe values can exceed the speed byte limit, and write() errors go undetected

## Describe the bug

I found two bugs in `src/drivers/roboclaw/Roboclaw.cpp` and reproduced both on a PC using an emulated UART.

### 1. Speed byte exceeds 127

The RoboClaw Packet Serial commands 0, 1, 4 and 5 expect a data byte between 0 and 127 (RoboClaw User Manual, Rev. 6.0, pp. 48–49).

The driver calculates the byte in `updateOutputs()` and converts it in `sendUnsigned7Bit()`. However, `module.yaml` allows failsafe values up to 257, and the mixer passes the configured value to the driver unchanged during flight termination.

This means values above 254 produce bytes outside the expected range:

| Failsafe value | Data byte |
|---:|---:|
| 254 | 127 |
| 255 | 128 |
| 256 | 129 |
| 257 | 130 |

I haven't tested the behavior on actual RoboClaw hardware.

### 2. write() errors are not detected

In `writeCommand()` and `writeCommandWithPayload()`, the return value of `write()` is stored in a `size_t`.

When `write()` returns `-1`, the value becomes a large unsigned number. As a result, the check `bytes_written < 2` does not detect the error, and the function returns `OK`.

The caller then waits for a response even though the write failed. The transaction eventually times out in `select()` instead of reporting the original write error.

## Reproduction

Both tests use the original `Roboclaw.cpp`, compiled on a PC with the serial operations replaced by an emulated UART.

### Bug 1: Speed byte exceeds 127

**Input preparation:** Set `RBCLW_FAIL1` to 257. During flight termination, the mixer passes this value to the driver unchanged, so `outputs[0]` is 257.

```cpp
float outputs[2] = {257.f, 127.f};
```

**Function execution:** Call `dev.updateOutputs(outputs, 2, 1)`.

**Assertion violated:** The emulated `write()` checks that the data byte for commands 0, 1, 4 and 5 does not exceed 127. The driver sends 130, causing the assertion to fail.

### Bug 2: write() errors are not detected

**Input preparation:** Configure the emulated `write()` to return `-1` with `errno = EIO`.

```cpp
g_write_fails = true;
```

**Function execution:** Call `dev.readEncoder()`.

**Assertion violated:** The emulated `select()` checks whether a response is being awaited after a failed write. The assertion fails because the driver proceeds to `select()` instead of handling the write error.

With the fixes applied, both tests complete without triggering the assertions. The full test program is available at `<LINK-DO-ARTEFATO>`.

## Checks for existing protections

I traced both code paths to check whether another part of the driver prevents these issues.

For the speed byte, the normal output-limiting path uses `output_limit_calc()`, but the flight termination path passes the failsafe value directly to the driver. `sendUnsigned7Bit()` does not clamp the value either. Unlike this function, `sendSigned16Bit()` uses `math::constrain()`.

The configured failsafe range also accepts values from 0 to 257. Flight termination can be triggered while armed through `MAV_CMD_DO_FLIGHTTERMINATION` or failsafe actions configured as `Terminate`, such as `NAV_RCL_ACT` and `NAV_DLL_ACT`.

For the write error, I found no check between `write()` and the subsequent response wait that handles a failed write. Since `-1` is converted to an unsigned value, the existing size check does not catch it.

## Expected behavior

- **Speed byte:** The driver should ensure that commands 0, 1, 4 and 5 never send a data byte above 127. This could be handled by clamping the value in `sendUnsigned7Bit()` or by reducing the failsafe parameter maximum to 254.
- **Write errors:** The driver should return `ERROR` when `write()` fails. Storing the return value in an `ssize_t` and checking for a negative result before checking the packet length would address this issue.

## Software version

PX4-Autopilot commit `8be2590c1529517ccd54fbc09ae138402517baf4`. The same code was present on `main` when checked on 2026-10-19.

## Flight controller / Vehicle type

Rover with Roboclaw. Both bugs were reproduced on a PC using an emulated UART; no flight log or hardware reproduction is available.
