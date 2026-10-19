<!--
RASCUNHO do pull request com a correção, para a Marcela reescrever com as próprias palavras.
Abra o PR SÓ DEPOIS da issue, e cite a issue no texto (troque #NNNNN pelo número dela).
Como abrir: fork do PX4-Autopilot no seu GitHub → branch nova → aplicar correcao_roboclaw.patch
→ commit → "Compare & pull request" para PX4/PX4-Autopilot:main. Passo a passo no LEIA-ME.md (seção 6).
O PX4 usa o modelo de PR com as seções abaixo (Solved Problem / Solution / Changelog Entry / Test coverage).
-->

# Title

drivers/roboclaw: clamp drive command data byte to 0..127 and handle write() errors

## Solved Problem

Fixes #NNNNN.

1. `sendUnsigned7Bit()` could send a data byte above 127 in the Packet Serial drive commands 0, 1, 4 and 5, whose valid range is 0–127 (RoboClaw User Manual Rev. 6.0, pp. 48–49). This happens when the output value is above 254. The value range in `module.yaml` allows this for the failsafe value (up to 257), which `MixingOutput` forwards unchanged during flight termination.
2. In `writeCommand()` and `writeCommandWithPayload()`, the return value of `write()` was stored in a `size_t`. A write error (-1) became a very large number, so it was treated as a successful write.

## Solution

- `sendUnsigned7Bit()`: limit `fabsf(data)` to [0, 1] before scaling, so the byte is always 0–127.
- `writeCommand()` / `writeCommandWithPayload()`: store the result of `write()` in `ssize_t`, so -1 is detected as an error. The log message now prints the packet size instead of the payload size.

The failsafe range in `module.yaml` is left unchanged; it can also be reduced to 254 if maintainers prefer.

## Changelog Entry

```
Bugfix: Roboclaw driver no longer sends drive data bytes above 127 (e.g. failsafe values 255–257) and now reports write() errors
```

## Test coverage

- Formal verification with ESBMC 8.5.0 on the driver source (harness that includes `Roboclaw.cpp`):
  - before the change: "data byte ≤ 127 for outputs in [0, 257]" FAILS; "select() is not called after write() returns -1" FAILS.
  - after the change: both properties hold (VERIFICATION SUCCESSFUL).
- Native run of the driver with a simulated UART, before and after the change: outputs 255/256/257 sent bytes 128/129/130 before and 127 after; with `write()` returning -1, `select()` was called 7 times before and 0 times after.
- Not tested on hardware.
