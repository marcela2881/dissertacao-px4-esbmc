<!--
RASCUNHO para a Marcela reescrever com as próprias palavras (não enviar assim).
Formato: modelo "Bug report" do PX4-Autopilot (GitHub → Issues → New issue → Bug report).
Trechos entre [[ ]] são para você preencher ou confirmar antes de enviar.
-->

# Title

Roboclaw driver: failsafe values above 254 send out-of-range speed bytes (128–130); write() errors are treated as success

## Describe the bug

While formally verifying the Roboclaw driver (`src/drivers/roboclaw`) with the bounded model checker ESBMC, I found two issues in the original source. I confirmed both by running the unmodified `Roboclaw.cpp` natively with the values from the counterexamples.

**1. Drive commands can carry a data byte above 127.**

The Packet Serial drive commands used by the driver are Drive Forward/Backwards M1/M2 (commands 0, 1, 4, 5). Their data byte must be in 0–127 [[Basicmicro RoboClaw User Manual, section …, page …]].

- `updateOutputs()` maps the output value with `(outputs[i] - 127) / 127` and does not clamp it (Roboclaw.cpp:161–162).
- `sendUnsigned7Bit()` then computes `lroundf(fabs(data) * INT8_MAX)` (Roboclaw.cpp:291).

For outputs in the normal mixer range [0, 254], the byte stays within 0–127; ESBMC proves this for that range. However, `module.yaml` allows the failsafe parameter up to 257 (`failsafe: { min: 0, max: 257 }`). In flight termination, `MixingOutput` forwards the failsafe value without limiting it (mixer_module.cpp:497–501, `actualFailsafeValue()`). So with `RBCLW_FAIL1/2` set to 255, 256 or 257, the driver sends data bytes of 128, 129 or 130:

| output | data (normalized) | byte sent |
|---|---|---|
| 254 | +1.00000 | 127 (ok) |
| 255 | +1.00787 | **128** |
| 256 | +1.01575 | **129** |
| 257 | +1.02362 | **130** |

[[If you tested on hardware, describe what the RoboClaw did with bytes 128–130. If not, say it was not tested.]]

**2. A failed `write()` is treated as a successful write.**

In `writeCommand()` (Roboclaw.cpp:380–385) and `writeCommandWithPayload()` (Roboclaw.cpp:332–338), the result of `write()` is stored in a `size_t`:

```cpp
size_t bytes_written = write(_uart_fd, buffer, 2);
if (bytes_written < 2) { ... return ERROR; }
return OK;
```

On error, POSIX `write()` returns -1. Converted to `size_t`, this becomes `SIZE_MAX`, so the `< 2` check is false and the function returns `OK`. The driver then waits for a response with `select()`/`read()`.

## To Reproduce

Issue 1 (no hardware needed; the arithmetic of lines 161 and 291):

```cpp
#include <cmath>
#include <cstdint>
#include <cstdio>
int main() {
    for (float out : {254.f, 255.f, 256.f, 257.f}) {
        float data = (out - 127.0f) / 127.f;                                  // Roboclaw.cpp:161
        uint8_t byte = static_cast<uint8_t>(lroundf(fabs(data) * INT8_MAX));  // Roboclaw.cpp:291
        printf("%.0f -> %u\n", out, byte);
    }
}
```

Output: `254 -> 127`, `255 -> 128`, `256 -> 129`, `257 -> 130`.

On a vehicle: [[set RBCLW_FAIL1 = 257, trigger flight termination, and observe the bytes on the UART — only if you actually tested it]].

Issue 2: make `write()` return -1, for example by disconnecting the UART or with a mocked `write()`. In the native run of the original `readEncoder()` with `write()` always returning -1, `readEncoder()` still called `select()` 7 times.

## Expected behavior

1. The data byte of commands 0/1/4/5 never exceeds 127. Possible fixes (maintainers' choice):
   - clamp in the driver, e.g. `math::constrain(fabsf(data), 0.f, 1.f)` in `sendUnsigned7Bit()`, or clamp the normalized value in `updateOutputs()`; and/or
   - limit `failsafe` in `module.yaml` to the same range as `max` (127–254) [[check whether 257 is intentional]].
2. A `write()` error (return value < 0) is reported as `ERROR`, e.g. by storing the result in `ssize_t` and checking `if (bytes_written < 0 || (size_t)bytes_written < packet_size)`.

## Software Version

PX4-Autopilot commit `8be2590c1529517ccd54fbc09ae138402517baf4`. The relevant code is unchanged on `main` as of [[2026-10-19 — confirm]].

## Flight controller / Vehicle type

Found by static analysis and native execution of the driver source; no flight log. Rover with Roboclaw ([[hardware tested, if any]]).

## Additional context

- Verification: ESBMC 8.5.0 (Boolector), harness that includes the original `Roboclaw.cpp`. Properties: data byte ≤ 127 for outputs in [0, 257] (fails) and in [0, 254] (holds).
- Harness, replay program and logs: [[link to your public repository/folder, if you choose to share it]].
- [[Optional: this was found as part of my master's research on formal verification of PX4 modules (PPGEE/UFAM).]]
