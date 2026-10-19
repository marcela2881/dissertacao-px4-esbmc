<!--
RASCUNHO para a Marcela reescrever com as próprias palavras.
Onde enviar: github.com/PX4/PX4-Autopilot → Issues → New issue → Bug report.
O título vai no campo "Title"; cada seção "##" corresponde a um campo do formulário.
Versão em português para entender cada parte: RASCUNHO_ISSUE_PT.md
-->

# Title

Roboclaw: failsafe values above 254 produce speed bytes above 127, and write() errors are treated as success

## Describe the bug

While formally verifying the Roboclaw driver (`src/drivers/roboclaw`) with the ESBMC model checker, I found two problems. I then confirmed both by compiling and running the unmodified `Roboclaw.cpp` with the values from the counterexamples.

### 1. Speed byte above 127

The driver uses the Packet Serial commands 0, 1, 4 and 5 (Drive Forward/Backwards M1/M2). According to the RoboClaw User Manual (Rev. 6.0, "Commands 0 - 7 Compatibility Commands", pp. 48–49), the data byte of these commands must be between 0 and 127.

The driver computes this byte as follows:

- `updateOutputs()`: `(outputs[i] - 127) / 127`, with no limit (Roboclaw.cpp, lines 161–162)
- `sendUnsigned7Bit()`: `lroundf(fabs(data) * INT8_MAX)` (Roboclaw.cpp, line 291)

In the normal output range (0 to 254), the byte stays between 0 and 127. However, `module.yaml` allows the failsafe value up to 257 (`failsafe: { min: 0, max: 257 }`). During flight termination, the mixer sends the failsafe value to the driver without limiting it (mixer_module.cpp, lines 497–501). As a result:

| RBCLW_FAILn | byte sent |
|---|---|
| 254 | 127 |
| 255 | **128** |
| 256 | **129** |
| 257 | **130** |

I have not tested on hardware what the RoboClaw does with these values.

### 2. write() errors treated as success

In `writeCommand()` (lines 380–385) and `writeCommandWithPayload()` (lines 332–338), the return value of `write()` is stored in a `size_t`:

```cpp
size_t bytes_written = write(_uart_fd, buffer, 2);
if (bytes_written < 2) { ... return ERROR; }
return OK;
```

When `write()` fails, it returns -1. Stored in a `size_t`, -1 becomes a very large number, so the check `bytes_written < 2` is false and the function returns `OK`. The driver then waits for a reply that will not come. In my test, with `write()` always failing, `readEncoder()` called `select()` 7 times.

## To Reproduce

Problem 1, without hardware (same arithmetic as lines 161 and 291):

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

## Expected behavior

1. The speed byte should never be above 127. Possible fixes:
   - limit the value in the driver, for example `math::constrain(fabsf(data), 0.f, 1.f)` in `sendUnsigned7Bit()`; or
   - change the failsafe limit in `module.yaml` from 257 to 254 (same as `max`).
2. A `write()` error should return `ERROR`, for example by storing the result in a `ssize_t` and checking for a negative value.

## Software Version

PX4-Autopilot commit `8be2590c1529517ccd54fbc09ae138402517baf4`. The same code is still on `main` (checked on 2026-10-19).

## Flight controller / Vehicle type

Rover with Roboclaw. Found by formal verification and by running the driver code on a PC; no flight log.

## Additional context

Tool: ESBMC 8.5.0. The verification includes the original `Roboclaw.cpp`. The property "byte ≤ 127" holds for outputs from 0 to 254 and fails for outputs up to 257.
