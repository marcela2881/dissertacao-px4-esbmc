# RoboClaw: drive commands can send values above 127, and `write()` errors go undetected

## Describe the bug

While analysing the RoboClaw driver, I found two issues in `src/drivers/roboclaw/Roboclaw.cpp`. In the first, drive commands can send values above the limit accepted by the protocol. In the second, a failed write can go unnoticed by the driver, which keeps waiting for a response even though the operation failed.

I reproduced both issues on a PC using an emulated UART. For the first one, I also ran a bench test with a Pixhawk 6X RT connected to an ESP32 emulating the RoboClaw. I did not use a physical RoboClaw device.

I found these issues in the previous implementation of the driver, present on `main` at commit `8be2590c1529517ccd54fbc09ae138402517baf4` (4 June 2026). I also confirmed that this implementation was still on `main` on 19 October 2026. On the `release/1.18` branch, checked at commit `8303efda43cb` on 12 November 2026, I found the same code: `Roboclaw.cpp`, `Roboclaw.hpp` and `module.yaml` are identical in the two revisions.

The driver was later refactored in PR #29473, merged on 28 October 2026. In the new implementation, the failsafe value is limited to 254, values are constrained before conversion, and the result of the write call is stored in an `ssize_t`. I have not tested the refactored driver. I am reporting these issues because the release branch still contains the previous implementation.

### 1. Drive commands can send values above 127

The first issue is in how drive commands are sent. According to the RoboClaw User Manual, Revision 6.0, pp. 48–49, commands 0, 1, 4 and 5 accept data values between 0 and 127.

Following the failsafe value down to the driver, I noticed that the parameters defined in `module.yaml` allow values up to 257. On flight termination, the mixer passes the configured value to the driver without limiting it to the range accepted by the protocol. The normal output path goes through `output_limit_calc()`, but the failsafe value does not. I also checked that `sendUnsigned7Bit()` does not enforce the range, unlike `sendSigned16Bit()`, which constrains its input.

As a result, failsafe values of 255, 256 and 257 are sent as data bytes 128, 129 and 130, respectively. In other words, the driver can transmit values above the limit for these commands.

### 2. `write()` errors go undetected

The second issue is in `writeCommand()` and `writeCommandWithPayload()`. Both functions store the return value of `write()` in a `size_t`, which is an unsigned type.

When `write()` returns `-1` to indicate a failure, the value is converted to a very large unsigned integer. Because of that, the size checks that follow do not catch the error, and the functions return `OK`.

In practice, the driver assumes the write succeeded and keeps waiting for a response from the RoboClaw, even though the command was not sent. The original error is hidden and only shows up later, when `select()` times out.

## How I reproduced it

To check whether the issues actually happen, I used the original `Roboclaw.cpp` on a PC and replaced the serial calls with an emulated UART.

For the first issue, I set `RBCLW_FAIL1` to 257 and ran the flight termination path with `outputs[0] = 257`. During `updateOutputs()`, the emulated UART received a data byte of 130 for commands 0, 1, 4 and 5.

For the second issue, I made the emulated `write()` return `-1` with `errno = EIO` and called `readEncoder()`. Even with the failed write, the driver did not detect the error and kept waiting for a response through `select()`.

The harnesses, commands and execution logs are available here:

https://github.com/marcela2881/dissertacao-px4-esbmc/tree/080955010dfc8585197c3b52e8c5d6cf34599e71/testes_hardware/roboclaw

### Bench test (first issue only)

After reproducing the first issue on the PC, I wanted to check whether the same behaviour also happens on a Pixhawk board. I built `px4_fmu-v6xrt_default` from commit `8be2590c15`, enabling `CONFIG_DRIVERS_ROBOCLAW=y` in the board configuration, since the driver is not enabled by default for this board, and flashed it to a Pixhawk 6X RT.

I used the following parameters:

- `RBCLW_SER_CFG=102` (TELEM2)
- `SER_TEL2_BAUD=38400`
- `RBCLW_FUNC1=101`
- `RBCLW_FAIL1=257`

I connected TELEM2 to an ESP32 set up to emulate the RoboClaw and answer the driver's queries.

After running `commander arm -f` and `commander termination`, I confirmed that `actuator_armed.termination` was `True`. At that point the Pixhawk started transmitting:

`80 00 82 8A 90`

This is address `0x80`, command 0, data byte `0x82` (130) and a valid CRC. The packet was sent approximately every 50 ms.

For comparison, I changed `RBCLW_FAIL1` to 254 and repeated the test. This time the Pixhawk transmitted:

`80 00 7F B4 22`

The data byte was 127, within the expected limit.

After applying my proposed fix, I set `RBCLW_FAIL1=257` again and repeated the test. The Pixhawk transmitted:

`80 00 7F B4 22`

During approximately four minutes of capture, the ESP32 did not record any data byte above 127.

The serial captures and the ESP32 sketch are in the `hardware_pixhawk6xrt` directory of the repository linked above.

### Note on UART initialization

During the bench tests I also ran into a behaviour worth mentioning. To get the driver running, I had to value-initialize the members `_uart_fd_set{}` and `_uart_fd_timeout{}` in `Roboclaw.hpp`.

Before this change, the boot log showed `Starting Roboclaw Driver on /dev/ttyS5` and no further message from the driver. In the two boots I recorded, `wq:hp_default` was starved, with its items running far below their expected rates:

- `battery_status`: 0.1 Hz, with 11–14 s intervals;
- `board_adc`: 0.2–0.3 Hz, where 10 ms was expected;
- `pwm_out`: 62–79 s intervals, where 300 ms was expected;
- `roboclaw`: 0.0 Hz.

The other work queues kept running at their normal rates.

With the two members initialized, the driver printed `Successfully connected` and `wq:hp_default` went back to normal, with `battery_status` at 99.9 Hz and `roboclaw` at 3.4 Hz.

Looking at the code, `Run()` calls `_mixing_output.update()` before `initializeUART()`. So the first `updateOutputs()` call can reach `select()` before `_uart_fd_set` and `_uart_fd_timeout` have been initialized. The object is allocated with `new`, and `_uart_fd` is still 0 at that point.

I have not confirmed with a debugger that this is exactly where the work queue blocks. What I observed is the difference before and after initializing the two members. The driver source differed only in those two lines, but I was also adjusting RoboClaw parameters between those boots, so this was not a fully controlled comparison. For that reason I am recording it as an observation based on the code and the bench results, not as a third issue with a confirmed cause.

The three console logs are in the same `hardware_pixhawk6xrt` directory.

## Formal verification

Besides the tests, I also analysed the original implementation with ESBMC 8.2.0 and CBMC 5.95.1. Both tools reported the two issues described above.

After applying the proposed fixes, I ran the verification again and both passed.

## Expected behavior

Commands 0, 1, 4 and 5 should only send values between 0 and 127.

When a call to `write()` fails, the driver should detect and report the error instead of returning `OK` and waiting for a response.

I have fixes for both issues and can open a PR against `release/1.18` if the maintainers consider them useful.

## Software version

- `release/1.18`, commit `8303efda43cb`: code inspected on 12 November 2026.
- PX4-Autopilot commit `8be2590c1529517ccd54fbc09ae138402517baf4`: code used for the analysis, the PC reproduction and the bench test.
- The same implementation was present on `main` on 19 October 2026.

## Flight controller / Vehicle type

Rover with RoboClaw.

I reproduced both issues on a PC using an emulated UART. I also observed the first one on a Pixhawk 6X RT, on the bench and without motors connected, with an ESP32 emulating the RoboClaw.

I have not tested with a physical RoboClaw device and do not have a flight log.

## AI assistance

I used a generative AI assistant to support parts of the analysis and the use of the verification tools. I checked the information against the source code and the test results. I am responsible for the content of this report.
