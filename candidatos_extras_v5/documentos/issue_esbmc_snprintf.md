<!--
RASCUNHO para o repositorio do ESBMC (github.com/esbmc/esbmc).
Decisao do Prof. Lucas. Antes de abrir: procurar issues existentes sobre
snprintf e conferir na versao mais recente do ESBMC.
Titulo sugerido:
snprintf model returns a wrong length when the %s argument is built at run time
-->

### Summary

When the `%s` argument of `snprintf` is a string filled at run time, the value that
ESBMC returns from `snprintf` does not match the length of that string. With a string
literal it does.

### Reproducer

```c
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
	char b[1200];
	char name[11];
	for (int i = 0; i < 10; i++) name[i] = 'A';
	name[10] = '\0';
	assert(strlen(name) == 10);              // passes
	int r = snprintf(b, sizeof(b), "%s", name);
	assert(r == (int)strlen(name));          // fails; the native libc gives r == 10
	return 0;
}
```

`esbmc sondagem_snprintf.c`: <colar a saida verbatim de resultados_*/5_logger/esbmc_sondagem.txt>

Native run: <colar resultados_*/5_logger/sondagem_libc.txt>

Observed with ESBMC 8.1.0 and 8.4.0. When the argument contents are symbolic, the return
value looks unconstrained.

### Why it matters

A harness that checks `uint8_t len = snprintf(...)` for truncation cannot exercise the
truncation, because the modelled return value is not tied to the string length.
