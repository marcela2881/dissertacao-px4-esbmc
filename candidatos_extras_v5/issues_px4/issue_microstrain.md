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

Checked on `main` at {{PX4_MAIN}} ({{PX4_MAIN_DATA}}); the code is the same as at 8be2590c15.

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
instead of assuming it (ESBMC {{ESBMC_V}}, CBMC {{CBMC_V}}).

ESBMC (`esbmc harness.cpp --unwind 200 --overflow-check`):

```
{{MS_ESBMC}}
```

CBMC (`cbmc harness.cpp --unwind 200 --unwinding-assertions --bounds-check --pointer-check`):

```
{{MS_CBMC}}
```

AddressSanitizer on a replay of lines 53-57 with 128 non-null bytes ({{CXX_V}}).
In the replay `_port` is a separate 128-byte heap block so that ASan can see the write.
In the driver `_port` is a member of the `MicroStrain` object, so the extra byte lands on
the next member of the same allocation:

```
{{MS_ASAN}}
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
