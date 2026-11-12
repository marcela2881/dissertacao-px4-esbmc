# RoboClaw: Drive commands can send data bytes above 127, and `write()` errors go undetected

## Describe the bug

**Affected versions:** `release/1.18` (verified at `8303efda43cb` on 12 November 2026) and the previous implementation on `main`, at commit `8be2590c1529517ccd54fbc09ae138402517baf4` (4 June 2026). I also verified that the same implementation was still present on `main` on 19 October 2026.

The RoboClaw driver was refactored in PR #29473, merged on 28 October 2026. In the refactored implementation, the failsafe range is limited to 254, values are constrained before conversion, and the return value of the write call is stored in an `ssize_t`. I have not tested the refactored driver. I am reporting these issues because the release branch still contains the previous implementation.

I found two issues in `src/drivers/roboclaw/Roboclaw.cpp`. I reproduced both on a PC using an emulated UART. I also observed the first issue on a Pixhawk 6X RT wired to an ESP32 emulating the RoboClaw. I have not tested with a physical RoboClaw device.

### 1. Drive commands can send values above 127

RoboClaw packet serial commands 0, 1, 4, and 5 expect data values between 0 and 127 (RoboClaw User Manual, Revision 6.0, pp. 48–49).

The issue is that the `module.yaml` failsafe parameters allow values up to 257. When flight termination is triggered, the mixer passes the configured failsafe value to the driver without limiting it to the expected range. The `sendUnsigned7Bit()` function also does not check or constrain the value.

For example, failsafe values of 255, 256, and 257 are transmitted as data bytes 128, 129, and 130, respectively. All three values are outside the supported range for these commands.

### 2. `write()` errors are not detected

There is also an issue in `writeCommand()` and `writeCommandWithPayload()`. Both functions store the return value of `write()` in a `size_t`.

When `write()` returns `-1`, the value is converted to a large unsigned integer. As a result, the subsequent size checks do not detect the error, and the functions return `OK`.

The driver then continues waiting for a response that will never arrive. The original write error is effectively hidden, and the problem only becomes visible later as a `select()` timeout.

## How I reproduced the issues

I used the original `Roboclaw.cpp` on a PC, replacing the serial calls with an emulated UART.

To reproduce the first issue, I set `RBCLW_FAIL1` to 257 and executed the flight termination path with `outputs[0] = 257`. During `updateOutputs()`, the emulated UART received a data byte of 130 for commands 0, 1, 4, and 5, exceeding the supported range.

To reproduce the second issue, I made the emulated `write()` return `-1` with `errno = EIO` and called `readEncoder()`. The driver did not detect the error and continued waiting for a response through `select()`.

The harnesses, commands, and execution logs are available here:

https://github.com/marcela2881/dissertacao-px4-esbmc/tree/67b35cb9bd31983d0aafb06f3e38d082fce9cbf1/testes_hardware/roboclaw

### Hardware test (first issue only)

I built `px4_fmu-v6xrt_default` from commit `8be2590c15`, with `CONFIG_DRIVERS_ROBOCLAW=y` added to the board configuration (the driver is not enabled by default for this board), and flashed it to a Pixhawk 6X RT.

I configured:

- `RBCLW_SER_CFG=102` (TELEM2)
- `SER_TEL2_BAUD=38400`
- `RBCLW_FUNC1=101`
- `RBCLW_FAIL1=257`

I connected TELEM2 to an ESP32 that responds to driver queries and acknowledges commands as a RoboClaw would.

After running `commander arm -f` and `commander termination`, with `actuator_armed.termination: True`, the Pixhawk transmitted:

`80 00 82 8A 90`

This corresponds to address `0x80`, command `0`, data byte `0x82` (130), and a valid CRC. The packet was transmitted approximately every 50 ms.

With `RBCLW_FAIL1=254`, the same firmware transmitted:

`80 00 7F B4 22`

The data byte was 127.

After applying the proposed fix and setting `RBCLW_FAIL1=257` again, the Pixhawk also transmitted:

`80 00 7F B4 22`

The emulator did not record any data byte above 127 during approximately four minutes of capture.

The serial captures and the ESP32 emulator sketch are available in the `hardware_pixhawk6xrt` directory in the repository linked above.

One additional observation: to run the driver on this board, I had to value-initialize two members in `Roboclaw.hpp` (`_uart_fd_set{}` and `_uart_fd_timeout{}`). Without this initialization, the driver blocked `wq:hp_default` during startup.

This change does not affect the output path. Based on the code, `Run()` calls `_mixing_output.update()` before `initializeUART()`. Therefore, the first `updateOutputs()` call can reach `select()` while `_uart_fd_set` and `_uart_fd_timeout` are still uninitialized (the object is allocated with `new`) and `_uart_fd` is still 0.

I did not save a work-queue trace showing the blockage, so I consider this a code-based observation rather than a measured result.

## Formal verification

I also analyzed the original implementation using ESBMC 8.2.0 and CBMC 5.95.1. Both tools detected the two issues.

After applying the proposed fixes, both verification runs passed.

## What I checked

For the first issue, I traced both the normal output path and the flight termination path. The normal path goes through `output_limit_calc()`, but the failsafe value reaches the driver without being constrained to the expected range. I also checked `sendUnsigned7Bit()` and confirmed that it does not enforce the 0–127 range. This differs from `sendSigned16Bit()`, which constrains its input.

For the second issue, I traced the return value of `write()` through the size checks and the code that waits for a response. I could not find any handling of a negative return value before the driver continues.

## Expected behavior

Data sent by commands 0, 1, 4, and 5 should always remain within the range 0–127.

If `write()` fails, the driver should report the error instead of returning `OK` and waiting for a response.

I have candidate fixes for both issues and would be happy to submit a PR against the release branch if the maintainers consider them useful.

## Software version

`release/1.18` at `8303efda43cb` (code inspection) and PX4-Autopilot commit `8be2590c1529517ccd54fbc09ae138402517baf4` (analysis, PC reproduction and hardware test). The same implementation was present on `main` on 19 October 2026.

## Flight controller / Vehicle type

Rover with RoboClaw.

Both issues were reproduced on a PC using an emulated UART. The first issue was also observed on a Pixhawk 6X RT (on the bench, without motors connected) with an ESP32 emulating the RoboClaw.

I have not tested with a physical RoboClaw device and do not have a flight log.

## AI assistance

I used a generative AI assistant to help with parts of the analysis and the verification tooling. I checked the claims against the source code and tool results, and I take responsibility for this report.
