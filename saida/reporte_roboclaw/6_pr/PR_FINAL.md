### Title

drivers/roboclaw: clamp drive command data byte to 0..127 and handle write() errors

### Solved Problem

Fixes #NNNNN.

This PR fixes two issues in the Roboclaw driver:

1. `sendUnsigned7Bit()` could send a data byte greater than 127 for Packet Serial drive commands 0, 1, 4 and 5. The valid range for these commands is 0–127 (RoboClaw User Manual Rev. 6.0, pp. 48–49). This can happen when the output is above 254. The failsafe range in `module.yaml` goes up to 257, and `MixingOutput` passes that value to the driver unchanged during flight termination.
2. In `writeCommand()` and `writeCommandWithPayload()`, the return value of `write()` was stored in a `size_t`. When `write()` returned -1, the value became a large unsigned number and the driver treated the write as successful.

### Solution

- `sendUnsigned7Bit()`: Limit `fabsf(data)` to the range [0, 1] before scaling it. This keeps the data byte between 0 and 127.
- `writeCommand()` / `writeCommandWithPayload()`: Store the return value of `write()` in a `ssize_t`, so a return value of -1 is correctly detected as an error. In `writeCommandWithPayload()`, the log message now prints the packet size instead of the payload size.

The failsafe range in `module.yaml` is unchanged. It could also be reduced to 254 if the maintainers prefer that approach.

### Changelog Entry

```
Bugfix: Roboclaw driver no longer sends drive data bytes above 127 (e.g. failsafe values 255–257) and now reports write() errors
```

### Test coverage

- Formal verification with ESBMC 8.5.0 on the driver source, using a harness that includes `Roboclaw.cpp`:
  - Before the change: "data byte ≤ 127 for outputs in [0, 257]" FAILS; "select() is not called after write() returns -1" FAILS.
  - After the change: both properties hold (VERIFICATION SUCCESSFUL).
  - Unrelated to this PR: with ESBMC's overflow check enabled, `swapBytesInt32()` is flagged at `buffer[0] << 24`. This is defined behavior in C++17 when the result fits in the unsigned type, so this PR does not change it.
- Native run of the driver with a simulated UART, before and after the change: outputs 255/256/257 sent bytes 128/129/130 before the fix and 127 after it. When `write()` returned -1, `select()` was called 7 times before the fix and 0 times after it.
- Not tested on hardware.
