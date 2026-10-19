### Title

Roboclaw: failsafe values above 254 produce speed bytes above 127, and write() errors are treated as success

### Describe the bug

While checking the Roboclaw driver (`src/drivers/roboclaw`) with the ESBMC model checker, I found two issues. I also reproduced both problems by compiling and running the original `Roboclaw.cpp` on a PC with a simulated UART, using the values from the counterexamples.

#### 1. Speed byte can be greater than 127

The driver uses Packet Serial commands 0, 1, 4 and 5 (Drive Forward/Backwards M1/M2). According to the RoboClaw User Manual (Rev. 6.0, "Commands 0 - 7 Compatibility Commands", pp. 48–49), the data byte for these commands must be between 0 and 127.

The driver calculates this byte in two steps:

- `updateOutputs()`: `(outputs[i] - 127) / 127`, without limiting the result (Roboclaw.cpp, lines 161–162)
- `sendUnsigned7Bit()`: `lroundf(fabs(data) * INT8_MAX)` (Roboclaw.cpp, line 291)

For outputs between 0 and 254, the byte stays within the valid range. However, `module.yaml` allows failsafe values up to 257 (`failsafe: { min: 0, max: 257 }`). During flight termination, the mixer passes the failsafe value to the driver without limiting it (mixer_module.cpp, lines 497–501).

This means the driver can send a byte greater than 127:

| RBCLW_FAILn | byte sent |
|---|---|
| 254 | 127 |
| 255 | **128** |
| 256 | **129** |
| 257 | **130** |

I have not tested on hardware what the RoboClaw does with these values.

#### 2. write() errors are treated as success

In `writeCommand()` (lines 380–385) and `writeCommandWithPayload()` (lines 332–338), the return value of `write()` is stored in a `size_t`:

```cpp
size_t bytes_written = write(_uart_fd, buffer, 2);
if (bytes_written < 2) { ... return ERROR; }
return OK;
```

When `write()` fails, it returns -1. Because the result is stored in a `size_t`, -1 becomes a very large unsigned number. The check `bytes_written < 2` therefore evaluates to false, and the function returns `OK` even though the write failed.

The driver then waits for a reply that will not arrive. In my test, when `write()` always returned an error, `readEncoder()` called `select()` 7 times.

### To Reproduce

Problem 1, without hardware (using the same calculations as lines 161 and 291):

```cpp
#include <cmath>
#include <cstdint>
#include <cstdio>
int main() {
    for (float out : {254.f, 255.f, 256.f, 257.f}) {
        float data = (out - 127.0f) / 127.f;
        uint8_t byte = static_cast<uint8_t>(lroundf(fabs(data) * INT8_MAX));
        printf("%.0f -> %u\n", out, byte);
    }
}
```

Output: `254 -> 127`, `255 -> 128`, `256 -> 129`, `257 -> 130`.

On a vehicle: set `RBCLW_FAIL1` to 255, 256 or 257 and trigger flight termination.

Problem 2: make `write()` return -1 (for example, by replacing `write()` in a test build).

### Expected behavior

1. The speed byte should never be greater than 127. Possible fixes:
   - Limit the value in the driver, for example with `math::constrain(fabsf(data), 0.f, 1.f)` in `sendUnsigned7Bit()`; or
   - Change the failsafe limit in `module.yaml` from 257 to 254 (the same as `max`).

2. If `write()` fails, the function should return `ERROR`. For example, store the return value in a `ssize_t` and check whether it is negative.

### Software Version

PX4-Autopilot commit `8be2590c1529517ccd54fbc09ae138402517baf4`. The same code is still on `main` (checked on 2026-10-19).

### Flight controller / Vehicle type

Rover with Roboclaw. The issues were found through formal verification and by running the driver code on a PC. No flight log is available.

### Additional context

Tool: ESBMC 8.5.0. The verification uses the original `Roboclaw.cpp`. The property "byte ≤ 127" holds for outputs from 0 to 254 but fails when the output range is extended to 257.
