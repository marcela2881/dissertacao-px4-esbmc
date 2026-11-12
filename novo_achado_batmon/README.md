# Novo achado: batmon, número de células × `voltage_cell_v[14]`

Conferido em 13/10/2026 contra o PX4 `main` c9b9e4e25e. **Ainda não reportado. Não publicar sem o Prof. Lucas.**

## O defeito

- `src/drivers/smart_battery/batmon/batmon.cpp:243`: `_cell_count = math::min((uint8_t)num_cells, (uint8_t)MAX_CELL_COUNT);`, com `MAX_CELL_COUNT = 16`.
- `batmon.cpp:143-145`: `for (i < _cell_count) new_report.voltage_cell_v[i] = _cell_voltages[i];`.
- `voltage_cell_v` é `float32[14]` (`msg/versioned/BatteryStatus.msg:38`).

O número de células vem do registrador SMBus 0x40 da própria bateria. O driver define registradores até a célula 16 (`BATT_SMBUS_CELL_16_VOLTAGE = 0x30`), então uma bateria **legítima** de 15S ou 16S já basta para disparar o defeito.

Efeito pelo layout da struct gerada:

- **15 células:** o valor extra cai em `max_cell_voltage_delta`, que é reescrito logo depois. Sem efeito visível.
- **16 células:** o valor extra cai em `full_charge_capacity_wh`, que o batmon não preenche. O campo é publicado com a tensão da 16ª célula (3,715 no replay). Além disso, `cell_count = 16` é publicado num array de 14.

O driver está nas configurações padrão de muitas placas: px4/fmu-v3, v4, v4pro, v5, v5x e v6u; cubepilot/cubeorange, cubeorangeplus e cubeyellow; micoair; entre outras.

## Contexto upstream

O mesmo defeito foi corrigido no driver UAVCAN/DroneCAN (`cbat_sub_cb`) pelo PR #28606, em 14/09/2026, relatado por @ttzero25. O batmon ficou de fora.

Buscas feitas em 13/10/2026 nas issues e PRs com "batmon", "cell_count", "voltage_cell_v" e "smbus" não encontraram nada sobre o batmon.

## Resultados (ambiente da nuvem)

| | código atual | `-DCORRECAO` (limitar ao tamanho do array) |
|---|---|---|
| ESBMC 8.1.0 (`--unwind 20 --overflow-check`) | FAILED (`_cell_count = 16`) | SUCCESSFUL |
| ESBMC 8.5.0 (`--multi-property`) | FAILED (2 de 13: `i < 14` e `array bounds violated`) | SUCCESSFUL |
| CBMC 5.95.1 (`--unwind 20 --unwinding-assertions --bounds-check --pointer-check`) | 1 de 15 falha | 0 de 15 |
| UBSan (`./replay 16`) | `index 14 out of bounds for type 'float [14]'` | — |
| replay sem sanitizador | `full_charge_capacity_wh=3.7150` com 16 células | — |

## Outros candidatos novos da mesma varredura (não reportados em 13/10/2026)

- **hott_sensors `recv_data()`:** laço sem limite gravando num buffer de 45 bytes na pilha, até chegar o byte 0x7d. É o mais grave, mas o driver é `default n` e não está em nenhuma placa.
- **pga460:** cada `read()` pede `sizeof(buf_rx)` e não o espaço restante, o que permite overflow na pilha. Driver fora de todas as placas.

Os candidatos de collision_prevention (#27826, #29030, #29037), SERIAL_CONTROL (#29032), RC_MAP_FLTMODE (#29067), sumd (#29031) e msp_osd (#28578) já foram reportados por outras pessoas.
