<!--
RASCUNHO PARA REVISAO DO PROF. LUCAS. NAO PUBLICAR SEM A APROVACAO DELE.
Este arquivo e gerado pelo rodar_tudo.sh em resultados_*/issues_prontas/,
com a saida verbatim dos logs da sua execucao no lugar dos {{...}}.
Antes de publicar: (1) conferir que o hash do main e o do dia;
(2) confirmar o paragrafo "AI assistance" (o que exatamente a IA fez) e a
decisao do Lucas sobre ele; (3) trocar <LINK-DO-ARTEFATO> pelo link do
repositorio/Zenodo com o pacote v5.
Titulo sugerido:
SUMD decoder writes the last data byte of a 32-channel frame one past sumd_data[64]
-->

### Summary

`sumd_decode()` stores data bytes starting at `sumd_data[1]` and keeps storing while
`_rxlen <= _rxpacket.length * 2`. `length` is accepted up to `SUMD_MAX_CHANNELS` (32),
and `sumd_data` has `SUMD_MAX_CHANNELS * 2` = 64 elements. For a 32-channel frame the
64th data byte is therefore written to `sumd_data[64]`, one past the end of the array.

Checked on `main` at {{PX4_MAIN}} ({{PX4_MAIN_DATA}}); the code is the same as at 8be2590c15.

### Affected code

`src/lib/rc/sumd.cpp`, state `SUMD_DECODE_STATE_GOT_LEN`:

```c
196:		_rxpacket.sumd_data[_rxlen] = byte;
...
207:		if (_rxlen <= ((_rxpacket.length * 2))) {
```

`src/lib/rc/sumd.h`:

```c
63:	uint8_t	sumd_data[SUMD_MAX_CHANNELS * 2];
64:	uint8_t	crc16_high;
```

`_rxlen` is 1 when the first data byte arrives (it is incremented when the length byte
is accepted), so data bytes go to indices 1..64. The channel extraction reads the same
range (`sumd_data[i * 2 + 1]` and `sumd_data[i * 2 + 2]`, line 345), so the indexing is
consistent but the array is one byte short.

### Effect

The struct is packed, so `sumd_data[64]` is `crc16_high`. The out-of-range byte lands
there and is then overwritten by the real CRC high byte in the next state. This stays
inside the same object, so it does not corrupt other memory, and AddressSanitizer does
not report it. UndefinedBehaviorSanitizer does.

Whether it changes any output depends on `max_chan_count`:

- `RCInput.cpp` (line 654) passes `input_rc_s::RC_INPUT_MAX_CHANNELS` (18). Channels above
  18 are not decoded, so the corrupted byte is never read. I found no functional effect
  on this path.
- With `max_chan_count = 32` (as in `src/lib/rc/rc_tests/RCTest.cpp`), channel 32 is
  decoded with the CRC high byte in place of its own low byte.

Running the real `sumd_decode()` (copied unchanged) on a valid 32-channel frame with a
correct CRC:

```
{{SUMD_EFEITO}}
```

### How it was found

Bounded model checking (ESBMC {{ESBMC_V}}, CBMC {{CBMC_V}}) on a harness that feeds the
decoder a frame with header, status and `length = 32` and free data bytes, then a
replay of the real function under UBSan ({{CXX_V}}).

ESBMC (`esbmc harness.cpp --unwind 70 --overflow-check`):

```
{{SUMD_ESBMC}}
```

CBMC (`cbmc harness.cpp --unwind 70 --unwinding-assertions --bounds-check --pointer-check`):

```
{{SUMD_CBMC}}
```

UBSan on the replay of the real `sumd_decode()`:

```
{{SUMD_UBSAN}}
```

Harness, replay, commands and full logs: <LINK-DO-ARTEFATO>

### Scope

- Reachable with a legitimate 32-channel SUMD frame; no malformed input is needed.
- In-object only (lands on `crc16_high`); no write outside `rc_decode_buf`.
- I did not test on hardware.

### Question

Is the 1-based indexing into `sumd_data` intended? If so, would you prefer the array to
hold 65 bytes, or the index to be shifted? I can open a PR with whichever you prefer.

### AI assistance

Parts of this work (reviewing the harnesses against the PX4 source and drafting this
text) were assisted by an AI tool (Claude, Anthropic). The verification results come from
the tools named above, and the outputs quoted here are verbatim.
