# Draft comment for the RoboClaw issue (to be reviewed by Prof. Lucas before posting)

Follow-up on the uninitialised `_uart_fd_set` / `_uart_fd_timeout` observation (release/1.18 only; `main` was refactored).

I reproduced the pattern outside PX4, in a small standalone program that mimics the order of calls in `Run()`: the mixer update (which reaches `readAcknowledgement()`) happens before `initializeUART()`, so `select()` receives an uninitialised `fd_set` and `timeval`.

Valgrind 3.22.0 reports 3 warnings, all originating in `readAcknowledgement()` called from `Run()`:
- 2x "Conditional jump or move depends on uninitialised value(s)" inside `select()`
- 1x "Syscall param pselect6(timeout) points to uninitialised byte(s)"

The variant that initialises both members before the first call is clean. MemorySanitizer did not flag it in this setup, so I am not relying on it.

Caveats: this is a reconstruction of the pattern, not the PX4 binary; how long `select()` blocks depends on what the dirty memory contains (I used 0x7f in the demo). On hardware (Pixhawk 6X RT, release/1.18) the console captures from the earlier test show the same symptom. I have not yet tested this on `main` after the refactor.

Logs and the program: `roboclaw_uninit_2026-11/` in the thesis repository.

I used an AI assistant to help prepare the reproduction; I reviewed and ran it myself.
