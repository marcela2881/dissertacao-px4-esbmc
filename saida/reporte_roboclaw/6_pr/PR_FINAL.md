### Title

drivers/roboclaw: clamp drive command data byte to 0..127 and detect write() errors

### Solved Problem

Fixes #NNNNN.

This PR fixes two bugs in the Roboclaw driver:

1. `sendUnsigned7Bit()` could send a data byte greater than 127 for Packet Serial drive commands 0, 1, 4 and 5, whose valid range is 0–127 (RoboClaw User Manual Rev. 6.0, pp. 48–49). This happens when the output is above 254: the failsafe range in `module.yaml` goes up to 257, and `MixingOutput` passes the failsafe value to the driver unchanged during flight termination. No other function on this path limits the value.
2. In `writeCommand()` and `writeCommandWithPayload()`, the return value of `write()` was stored in a `size_t`. When `write()` returned -1, it became a large unsigned number, the size check passed and the function returned `OK`. The failure was only noticed later, when `select()` timed out waiting for a reply, and the log showed a timeout instead of the write error.

### Solution

- `sendUnsigned7Bit()`: limit `fabsf(data)` to [0, 1] before scaling, so the data byte stays between 0 and 127 (the same approach `sendSigned16Bit()` already uses with `math::constrain()`).
- `writeCommand()` / `writeCommandWithPayload()`: store the return value of `write()` in a `ssize_t` and compare it with the packet size as `ssize_t`, so -1 is reported as an error. In `writeCommandWithPayload()`, the log message now prints the packet size instead of the payload size.

The failsafe range in `module.yaml` is unchanged. It could also be reduced to 254 if the maintainers prefer that approach.

### Changelog Entry

```
Bugfix: Roboclaw driver no longer sends drive data bytes above 127 (e.g. failsafe values 255–257) and now detects write() errors
```

### Test coverage

Tested on a PC by compiling the driver source with an emulated UART (the serial calls `write`, `read`, `select`, `open` and termios are replaced; `Roboclaw.cpp` is used as is). Each test prepares the input, runs a Roboclaw function and checks an assert:

| Test | Input | Function | Assert | Before | After |
|---|---|---|---|---|---|
| Bug 1 | `outputs[0] = 257` (failsafe in termination) | `updateOutputs()` | data byte of commands 0/1/4/5 ≤ 127 | violated (byte 130) | holds (byte 127) |
| Bug 2 | `write()` returns -1 | `readEncoder()` | `select()` is not called after a failed `write()` | violated | holds |

Outputs 255, 256 and 257 produced bytes 128, 129 and 130 before the change and 127 after it. Outputs from 0 to 254 produce the same bytes as before.

Not tested on hardware.
