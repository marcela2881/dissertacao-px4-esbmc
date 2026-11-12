# RoboClaw: drive commands can send a data byte above 127, and `write()` errors go undetected

## Describe the bug

I found two issues in `src/drivers/roboclaw/Roboclaw.cpp`. I first reproduced both on a PC using an emulated UART. I also tested the first issue on a bench setup with a Pixhawk 6X RT and an ESP32 emulating the RoboClaw device.

This applies to the driver as it is in `release/1.18`. The driver on `main` was refactored in #29473 (details under "Software version").

### 1. Drive commands can send a data byte above 127

According to the RoboClaw User Manual (Rev. 6.0, pp. 48–49), packet commands 0, 1, 4, and 5 accept data values from 0 to 127.

However, the `RBCLW_FAILn` parameters in `module.yaml` allow values up to 257. When flight termination is triggered, the configured failsafe value is passed to the driver without being limited to the protocol's 7-bit range.

The normal output path uses `output_limit_calc()`, but the failsafe path does not apply the same limitation. In addition, `sendUnsigned7Bit()` does not constrain the value before sending it, unlike `sendSigned16Bit()`.

As a result, failsafe values of 255, 256, and 257 can produce data bytes of 128, 129, and 130, respectively. These values are outside the range accepted by the RoboClaw protocol.

### 2. `write()` errors are treated as successful writes

In `writeCommand()` and `writeCommandWithPayload()`, the return value of `write()` is stored in a `size_t` variable.

When `write()` fails and returns `-1`, the value is converted to a large unsigned integer. Because of this, the subsequent size checks do not detect the error, and the functions return `OK`.

The driver then continues as if the command had been sent successfully and waits for a response that will never arrive. Eventually, `select()` times out.

## How I reproduced it

I reproduced both issues on a PC using the original `Roboclaw.cpp` implementation, replacing the serial communication calls with an emulated UART.

**Issue 1 — data byte above 127**

I set `RBCLW_FAIL1=257` and executed the flight termination path with `outputs[0] = 257`.

When `updateOutputs()` processed the failsafe value, the emulated UART received a data byte of `130` for commands 0, 1, 4, and 5.

**Issue 2 — undetected `write()` error**

I configured the emulated `write()` function to return `-1` with `errno=EIO` and then called `readEncoder()`.

The driver did not report the write failure. Instead, it continued and waited for a response until `select()` timed out.

The test code and supporting files are available here:

https://github.com/marcela2881/dissertacao-px4-esbmc/tree/080955010dfc8585197c3b52e8c5d6cf34599e71/testes_hardware/roboclaw

### Bench test

After reproducing the first issue on the PC, I wanted to check whether it also occurred on a Pixhawk board.

I built `px4_fmu-v6xrt_default` from commit `8be2590c15`, enabled `CONFIG_DRIVERS_ROBOCLAW=y` in the board configuration, and flashed the firmware to a Pixhawk 6X RT.

I connected TELEM2 to an ESP32 running a RoboClaw emulator and used the following parameters:

- `RBCLW_SER_CFG=102` (TELEM2)
- `SER_TEL2_BAUD=38400`
- `RBCLW_FUNC1=101`
- `RBCLW_FAIL1=257`

<!-- IMAGE: bench photo -->

After running `commander arm -f` and `commander termination`, I confirmed that `actuator_armed.termination` was `True`.

The ESP32 captured the following packet, approximately every 50 ms:

`80 00 82 8A 90`

The data byte is `0x82` (130), which is above the protocol limit of 127. The packet CRC was valid.

<!-- IMAGE 1: serial monitor, original firmware (80 00 82) -->

The emulator's counter of drive packets with a data byte above 127 kept increasing during this run (for example, `com velocidade >127=406` at 22:29:18 and `=3105` at 22:31:33, with `CRC ruim=0`, i.e. no CRC errors).

I then changed `RBCLW_FAIL1` to `254`. The captured packet became:

`80 00 7F B4 22`

In this case, the data byte was `0x7F` (127), which is within the protocol range.

After applying the proposed fix and setting `RBCLW_FAIL1=257` again, the captured packet remained:

`80 00 7F B4 22`

I monitored the output for approximately four minutes and did not observe any data byte above 127.

<!-- IMAGE 2: serial monitor, with the proposed fix (80 00 7F) -->

In this run the same counter stayed at `com velocidade >127=0` from the first to the last summary line.

The serial captures and the ESP32 sketch are included in the `hardware_pixhawk6xrt` folder.

This test was performed on a bench, with the ESP32 emulating the RoboClaw. I did not use a physical RoboClaw device or run a flight test.

### Additional observation: UART initialization

While setting up the bench test, I noticed another behavior related to UART initialization.

I value-initialized `_uart_fd_set` and `_uart_fd_timeout` in `Roboclaw.hpp`. Before this change, the boot log showed:

`Starting Roboclaw Driver on /dev/ttyS5`

but no further driver messages appeared. In two boots, I also observed unusually low activity in `wq:hp_default`. The `roboclaw` work item showed 0.0 Hz, while other work items in the same queue had much lower rates than expected.

After initializing those two members, the driver printed:

`Successfully connected`

and the work queue returned to normal activity. The `roboclaw` work item then showed 3.4 Hz.

Looking at the code, `Run()` calls `_mixing_output.update()` before `initializeUART()`. This means the first call to `updateOutputs()` may reach `select()` before `_uart_fd_set` and `_uart_fd_timeout` have been initialized.

I have not confirmed with a debugger that this is exactly where the work queue blocks. Also, the RoboClaw parameters were adjusted between the boots, so I cannot attribute the behavior exclusively to this initialization change. I am including the observation because it may be useful when investigating the driver. The three console logs are in the same `hardware_pixhawk6xrt` folder.

## Formal verification

I also checked both issues using ESBMC 8.2.0 and CBMC 5.95.1.

Both tools reported violations for the original implementation. After applying the proposed fixes, the corresponding checks passed.

These results apply to the harnesses and properties used in the analysis; they do not establish that the entire driver is free of defects.

## Expected behavior

- Commands 0, 1, 4, and 5 should never send a data byte greater than 127, even when the configured failsafe value exceeds that limit.
- If `write()` fails, the driver should detect the error and return an error status instead of reporting success and waiting for a response.

<!-- ===== PART 2: paste from here into "Flight Log / Additional Information" ===== -->

## Software version

I checked the following revisions:

- `main`: commit `8be2590c1529517ccd54fbc09ae138402517baf4` (June 4, 2026).
- `release/1.18`: commit `8303efda43cb` (checked November 12, 2026).

The relevant files (`Roboclaw.cpp`, `Roboclaw.hpp`, and `module.yaml`) are identical in these two revisions.

The driver was refactored in PR #29473, merged on October 28, 2026. From reading the refactored code, it limits the failsafe value to 254, constrains values before conversion, and stores the result of the write call in an `ssize_t`. I have not tested the refactored driver.

I am reporting these issues because the older implementation is still present in `release/1.18`.

## Flight controller / Vehicle type

- Flight controller: Pixhawk 6X RT
- Vehicle type: the RoboClaw driver is used on rovers; the bench board was running the default quadcopter airframe (`4001_quad_x`).
- Bench setup: Pixhawk 6X RT connected to an ESP32 emulating the RoboClaw device
- No motors were connected during the bench test.

## AI assistance

I used a generative AI assistant to help with parts of the code analysis and the use of the verification tools. I reviewed the findings against the source code and the test results. I am responsible for the content of this report.
