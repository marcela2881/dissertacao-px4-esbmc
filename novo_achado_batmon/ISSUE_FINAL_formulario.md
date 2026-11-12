<!-- Texto para o formulário "Bug report" do PX4, dividido nos dois campos. Conferido contra o main de 2026-11-11 (8e94784e1dc0). -->

TÍTULO:
[Bug] batmon: the cell count can be higher than voltage_cell_v[14], so the driver writes past the array

<!-- ===== PARTE 1: campo "Describe the bug" ===== -->

The batmon smart battery driver can write one or two cell voltages past the end of `voltage_cell_v`. The number of cells is read from the battery and capped at `MAX_CELL_COUNT`, which is 16:

```cpp
_cell_count = math::min((uint8_t)num_cells, (uint8_t)MAX_CELL_COUNT);
```

The problem is that `voltage_cell_v`, in `battery_status_s`, only holds 14 floats (`msg/versioned/BatteryStatus.msg`). So the copy loop in `RunImpl()`:

```cpp
for (int i = 0; i < _cell_count; i++) {
    new_report.voltage_cell_v[i] = _cell_voltages[i];
}
```

goes out of bounds as soon as the battery reports 15 or 16 cells. The count comes straight from the battery over SMBus (register `0x40`), and the driver already defines voltage registers up to `BATT_SMBUS_CELL_16_VOLTAGE`, so you don't need a malformed response to hit this — a real 15S or 16S pack is enough.

The write isn't silent, because of how the generated struct is laid out. With 16 cells the extra voltage overwrites `full_charge_capacity_wh` (in my test it became `3.715`, the value I used for the last cell), and `cell_count` is still published as 16 even though only 14 voltages fit. With 15 cells the extra write lands on `max_cell_voltage_delta`, which is reassigned a few lines later, so that case ends up harmless.

## How I reproduced it

I checked the copy loop with ESBMC and CBMC, and wrote a short replay to see the effect at run time, built with UBSan and ASan:

- ESBMC (8.1.0 and 8.5.0) and CBMC (5.95.1) report a bounds violation on the original loop, and pass once the count is clamped to the array size.
- The replay is clean with 14 cells; with 15 or 16 it triggers UBSan (`index 14 out of bounds for type 'float[14]'`).
- With 16 cells the replay ends with `full_charge_capacity_wh = 3.715`.

I used `--unwind 20 --overflow-check` for ESBMC and `--unwind 20 --unwinding-assertions --bounds-check --pointer-check` for CBMC. The harness, the replay and the full logs are available here: https://github.com/marcela2881/dissertacao-px4-esbmc/tree/b51fcb271055e8bd3e07aaaa97409d45e91adbbc/novo_achado_batmon.

Before writing this up I looked for anything that already guards the copy. The only limit is against `MAX_CELL_COUNT` (16); there is nothing comparing the count to the 14 slots in `voltage_cell_v`. The same kind of overflow was fixed on the UAVCAN/DroneCAN side in #28606, but batmon was left as it is, and I couldn't find an existing issue or PR about this specific path (I searched "batmon", "cell_count", "voltage_cell_v" and "smbus").

## Expected behavior

The driver should never write past `voltage_cell_v`, and the `cell_count` it publishes should match the number of voltages that actually fit. The fix is a one-line clamp of the loop bound to the array size; I can open a PR if that is useful.

<!-- ===== PARTE 2: campo "Flight Log / Additional Information" ===== -->

No flight log: this was reproduced with ESBMC, CBMC and UBSan on a PC, not on hardware.

## Software version

I first checked commit `c9b9e4e25e`, which was `main` on 2026-10-13. I checked again on `main` at `8e94784e1dc0` (2026-11-11): `src/drivers/smart_battery/batmon` and `msg/versioned/BatteryStatus.msg` are unchanged between the two.

## Flight controller / Vehicle type

Any setup using the batmon SMBus smart battery driver with a 15S or 16S pack. I have not tested this on hardware.

## AI assistance

Parts of this analysis and the verification harness were prepared with the help of a generative AI assistant. I verified every claim above against the source code and the tool outputs, and I take responsibility for the content of this report.
