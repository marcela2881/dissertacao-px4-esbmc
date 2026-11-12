<!--
RASCUNHO PARA REVISAO DO PROF. LUCAS. NAO PUBLICAR SEM A APROVACAO DELE.
Este arquivo e gerado pelo rodar_tudo.sh em resultados_*/issues_prontas/,
com a saida verbatim dos logs da sua execucao no lugar dos {{...}}.
Antes de publicar: (1) conferir que o hash do main e o do dia;
(2) confirmar o paragrafo "AI assistance" (o que exatamente a IA fez) e a
decisao do Lucas sobre ele; (3) trocar <LINK-DO-ARTEFATO> pelo link do
repositorio/Zenodo com o pacote v5.
Titulo sugerido:
MicroStrain constructor writes the terminator one past _port[128] for a 128+ character device path
-->

### Summary

In the `MicroStrain` constructor, `max_len` can be equal to `sizeof(_port)` (128), and the
terminator is then written at `_port[128]`, one past the end of the array.

Checked on `main` at 3fc3b141d7 (2026-10-12); the code is the same as at 8be2590c15.

### Affected code

`src/drivers/ins/microstrain/MicroStrain.cpp`:

```c
52:	memset(_port, '\0', sizeof(_port));
53:	const size_t max_len = math::min(sizeof(_port), strnlen(uart_port, sizeof(_port)));
54:	strncpy(_port, uart_port, max_len);
...
57:	_port[max_len] = '\0';
```

`MicroStrain.hpp` declares `char _port[128];`. If the first 128 characters of `uart_port`
contain no `'\0'`, `strnlen` returns 128, `min` returns 128, and line 57 writes
`_port[128]`. The `strncpy` itself is fine; only the terminator is out of range.

`uart_port` is the `-d` argument of `microstrain start` (line 1942), passed to
`new MicroStrain(dev)` at line 1961.

### How it was found

Bounded model checking with `uart_port` left free byte by byte and `strnlen`/`min`
computed as in the driver, so the verifier derives `max_len = 128` from the input
instead of assuming it (ESBMC 8.1.0, CBMC 5.95.1).

ESBMC (`esbmc harness.cpp --unwind 200 --overflow-check`):

```
Counterexample (last state before the violation):
  max_len = 128 (00000000 00000000 00000000 00000000 00000000 00000000 00000000 10000000)

Violated property:
  file harness.cpp line 112 column 5 function microstrain_init_port
  assertion max_len < PORT_SIZE
  max_len < 128


VERIFICATION FAILED
```

CBMC (`cbmc harness.cpp --unwind 200 --unwinding-assertions --bounds-check --pointer-check`):

```
[microstrain_init_port.assertion.1] line 112 assertion max_len < PORT_SIZE: FAILURE
[microstrain_init_port.array_bounds.2] line 114 array '_port' upper bound in _port[(signed long int)max_len]: FAILURE
** 2 of 30 failed (2 iterations)
```

AddressSanitizer on a replay of lines 53-57 with 128 non-null bytes (g++ (Ubuntu 13.3.0-6ubuntu2~24.04.1) 13.3.0).
In the replay `_port` is a separate 128-byte heap block so that ASan can see the write.
In the driver `_port` is a member of the `MicroStrain` object, so the extra byte lands on
the next member of the same allocation:

```
==7481==ERROR: AddressSanitizer: heap-buffer-overflow on address 0x50c0000000c0 at pc 0x55e1b45cc59f bp 0x7ffdaf8f75e0 sp 0x7ffdaf8f75d0
WRITE of size 1 at 0x50c0000000c0 thread T0
0x50c0000000c0 is located 0 bytes after 128-byte region [0x50c000000040,0x50c0000000c0)
SUMMARY: AddressSanitizer: heap-buffer-overflow /tmp/claude-0/-home-user-dissertacao-px4-esbmc/891a91a2-4eef-54c5-b3e5-07b3737105ab/scratchpad/v5/resultados_teste_nuvem/8_MicroStrain/fontes/replay.cpp:97 in main
```

Harness, replay, commands and full logs: <LINK-DO-ARTEFATO>

### Scope

- Needs a device path of 128 or more characters on the command line or in a startup
  script. Normal paths such as `/dev/ttyS2` are far shorter.
- One byte, written inside the object. I did not check which member follows `_port` in
  every build configuration, and I did not test on hardware.

### Question

Would you take a change that sizes the copy to `sizeof(_port) - 1` (or rejects longer
paths)? I can open a PR in whichever form you prefer.

### AI assistance

Parts of this work (reviewing the harnesses against the PX4 source and drafting this
text) were assisted by an AI tool (Claude, Anthropic). The verification results come from
the tools named above, and the outputs quoted here are verbatim.
