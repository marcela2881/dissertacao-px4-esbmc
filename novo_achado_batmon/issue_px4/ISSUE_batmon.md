# Title

batmon: cell count can exceed `voltage_cell_v[14]` and cause an out-of-bounds write

## Describe the bug

While checking the `batmon` smart battery driver, I noticed that the number of
cells reported by the battery can exceed the size of
`battery_status_s::voltage_cell_v`, which is an array of 14 floats.

In `batmon.cpp`, the cell count read from the battery over SMBus is limited to
`MAX_CELL_COUNT`, which is set to 16:

```
_cell_count = math::min((uint8_t)num_cells, (uint8_t)MAX_CELL_COUNT);
```

Later, in `RunImpl()`, the driver copies the cell voltages into the published
battery status:

```
for (int i = 0; i < _cell_count; i++) {
    new_report.voltage_cell_v[i] = _cell_voltages[i];
}
```

The problem is that `voltage_cell_v` has only 14 entries, as defined in
`BatteryStatus.msg`, while `_cell_count` can reach 16. If the battery reports
15 or 16 cells, the loop writes past the end of the array.

The cell count comes directly from the battery through SMBus register `0x40`.
The driver also defines cell voltage registers up to
`BATT_SMBUS_CELL_16_VOLTAGE`, so this is not necessarily caused by invalid
input. A battery with 15 or 16 cells can trigger the issue.

I also checked the generated struct layout. With 16 cells, the out-of-bounds
write can overwrite `full_charge_capacity_wh` with the voltage of the last
cell. In my reproduction, this field was published as `3.715` instead of `0`.
The reported `cell_count` remains 16, even though the array only has space for
14 cell voltages.

## Reproduction

I checked the loop using ESBMC and CBMC and reproduced the out-of-bounds access
with UBSan using a small replay of the same copy loop.

Results:

* UBSan: 14 cells run without an error. With 15 or 16 cells, it reports an
  out-of-bounds access at index 14.
* 16 cells: `full_charge_capacity_wh` is overwritten with `3.715`, the voltage
  of the last cell.
* ESBMC 8.2.0: Verification fails with `_cell_count = 16` and the assertion
  `i < 14`. After applying the fix, verification succeeds.
* CBMC 5.95.1: The bounds check fails before the fix and passes after it.

The checks were run with `--unwind 20 --overflow-check` for ESBMC and
`--unwind 20 --unwinding-assertions --bounds-check --pointer-check` for CBMC.

## Existing protections

I traced the cell count through the driver and found that it is only limited by
`MAX_CELL_COUNT` (16). There is no check against the actual size of
`voltage_cell_v` before the copy loop.

A similar issue was addressed in the UAVCAN/DroneCAN battery path
(`cbat_sub_cb`) in PR #28606, where the cell count was limited to the size of
`voltage_cell_v`. The `batmon` driver does not appear to have the same
protection.

I also searched existing issues and pull requests for `batmon`, `cell_count`,
`voltage_cell_v`, and `smbus`, but did not find an existing report of this
problem.

## Expected behavior

The driver should never write beyond the end of `voltage_cell_v`.

The cell count should be limited to the smallest of `num_cells`,
`MAX_CELL_COUNT`, and the actual number of entries in `voltage_cell_v` before
the copy loop runs. This would follow the same approach used in PR #28606 and
prevent the out-of-bounds write.

## Software version

PX4-Autopilot commit `8be2590c15`. I also confirmed the issue on `main` on
2026-10-13.

The driver is enabled by default on several boards, including px4/fmu-v3, v4,
v4pro, v5, v5x, and v6u; cubepilot/cubeorange, cubeorangeplus, and cubeyellow;
and micoair boards, among others.

## Flight controller / Vehicle type

Any vehicle using the `batmon` SMBus smart battery driver with a 15S or 16S
battery pack.

The issue was reproduced using ESBMC, CBMC, and UBSan on a PC. I do not have a
flight log for this case.
