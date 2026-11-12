# RoboClaw: drive commands can send a data byte above 127, and write() errors go undetected

## Describe the bug

I found two issues in `src/drivers/roboclaw/Roboclaw.cpp`. I reproduced both on a PC using an emulated UART. I also observed the first one on the wire on a Pixhawk 6X RT, with an ESP32 standing in for the RoboClaw. I haven't tested with a physical RoboClaw device.

**1. Drive commands can send values above 127**

Packet Serial commands 0, 1, 4 and 5 expect values between 0 and 127 (RoboClaw User Manual, Rev. 6.0, pp. 48–49).

The problem is that the failsafe parameters in `module.yaml` allow values up to 257. When flight termination is triggered, the mixer passes the configured value directly to the driver, without limiting it to the expected range. The `sendUnsigned7Bit()` function does not check the value either.

For example, a failsafe value of 255 is sent as 128, 256 as 129, and 257 as 130. All three values are outside the range supported by these commands.

**2. write() errors are not detected**

There is also a problem in `writeCommand()` and `writeCommandWithPayload()`. Both functions store the return value of `write()` in a `size_t`. When `write()` returns -1, the value is converted to a large unsigned number. As a result, the `bytes_written < 2` check does not catch the error, and the function returns OK.

The driver then continues waiting for a response that will never arrive. The original write error is effectively hidden, and the problem only becomes visible later as a timeout in `select()`.

## How I reproduced it

I used the original `Roboclaw.cpp` on a PC, replacing the serial calls with an emulated UART.

To reproduce the first issue, I set `RBCLW_FAIL1` to 257 and ran the flight-termination path with `outputs[0] = 257`. During `updateOutputs()`, the emulated UART received a data byte of 130 for commands 0, 1, 4 and 5, which exceeds the allowed range.

For the second issue, I made the emulated `write()` return -1 with `errno = EIO` and called `readEncoder()`. The driver did not detect the error and continued waiting for a response through `select()`.

The harnesses, commands and execution logs are available here: https://github.com/marcela2881/dissertacao-px4-esbmc/tree/b7c3832dc276decd3b825f4795eaf9927f9a9618/testes_hardware/roboclaw

**On hardware (first issue only).** I flashed a Pixhawk 6X RT with `px4_fmu-v6xrt_default`, set `RBCLW_SER_CFG=102` (TELEM2), `SER_TEL2_BAUD=38400`, `RBCLW_FUNC1=101` and `RBCLW_FAIL1=257`, and connected TELEM2 to an ESP32 that answers the driver's queries and ACKs its commands like a RoboClaw would. After `commander arm -f` and `commander termination` (`actuator_armed.termination: True`), the board sent `80 00 82 8A 90` every ~50 ms: address 0x80, command 0, data byte 0x82 (130), valid CRC. The capture and the emulator sketch are in the `hardware_pixhawk6xrt` folder of the link above.

One caveat: to get the driver running on this board I had to value-initialize two members in `Roboclaw.hpp` (`_uart_fd_set{}` and `_uart_fd_timeout{}`). Without that, the driver blocked `wq:hp_default` at startup. This change does not touch the output path. I can describe that startup problem separately if it is useful.

I also ran the original code through ESBMC 8.2.0 and CBMC 5.95.1. Both tools detected the two issues. After applying the fixes, both verification checks passed.

## What I checked

For the first issue, I followed the normal output path and the flight-termination path. The normal path goes through `output_limit_calc()`, but the failsafe value reaches the driver without being limited. I also checked `sendUnsigned7Bit()` and confirmed that it does not enforce the 0–127 range. This is different from `sendSigned16Bit()`, which applies a constraint to the value.

For the second issue, I traced the return value of `write()` through the size check and the code that waits for a response. I couldn't find any handling of a negative return value before the driver continues.

## Expected behavior

The data sent by commands 0, 1, 4 and 5 should always stay within the 0–127 range. If `write()` fails, the driver should report the error instead of returning OK and waiting for a response.

I have candidate fixes for both issues and would be happy to submit a PR if the maintainers consider it useful.

## Software version

PX4-Autopilot commit `8be2590c1529517ccd54fbc09ae138402517baf4`. I also checked that the same code was present on `main` on 2026-10-19.

## Flight controller / Vehicle type

Rover with RoboClaw. Both issues were reproduced on a PC using an emulated UART. The first was also observed on a Pixhawk 6X RT (bench, no motors) with an ESP32 emulating the RoboClaw. I haven't tested with a physical RoboClaw and don't have a flight log.

## AI assistance

I used a generative AI assistant to help with parts of the analysis and the verification harnesses. I checked the claims against the source code and the tool outputs, and I take responsibility for the report.
