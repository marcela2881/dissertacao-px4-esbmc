# Mapeamento dos 10 testes seeded — Boolector vs Z3
Objetivo: replicar a divergência ESBMC/Boolector vs CBMC/SAT observada no GPS,
em casos sintéticos controlados.
## Comandos para rodar cada teste
Para cada arquivo `.c`, rode com os dois solvers:
```bash
esbmc <arquivo>.c --unwind 10 --no-unwinding-assertions --overflow-check --default-solver boolector --timeout 120s
esbmc <arquivo>.c --unwind 10 --no-unwinding-assertions --overflow-check --default-solver z3 --timeout 120s
```
## Tabela de mapeamento (preencher após rodar)
| # | Arquivo | Padrão testado | Boolector | Z3 | Divergência? | Tempo (Boolector / Z3) |
|---|---|---|---|---|---|---|
| 1 | test_size_50.c | Buffer 50, single-shot | | | | |
| 2 | test_size_100.c | Buffer 100, single-shot | | | | |
| 3 | test_size_200.c | Buffer 200 (réplica GPS), single-shot | | | | |
| 4 | test_size_500.c | Buffer 500, single-shot | | | | |
| 5 | test_size_1000.c | Buffer 1000, single-shot | | | | |
| 6 | test_size_2000.c | Buffer 2000, single-shot | | | | |
| 7 | test_loop_unwind10.c | Loop, dump_len inicia em nondet, acumula | | | | |
| 8 | test_loop_corrupted_start.c | Loop, dump_len já nondet (estado corrompido) | | | | |
| 9 | test_uint16_width.c | Largura 16-bit em vez de size_t | | | | |
| 10 | test_double_guard_unprotected.c | Padrão MAVLink sem proteção (deliberado) | | | | |
**Preencher cada célula com:** `FAILED` (achou o bug) ou `SUCCESSFUL` (não achou) + tempo em segundos.
## Como interpretar
- Se **Boolector = FAILED** e **Z3 = SUCCESSFUL** no mesmo caso -> confirma a divergência do GPS, reforça a hipótese do paper.
- Se **ambos = FAILED** ou **ambos = SUCCESSFUL** -> não há divergência nesse caso específico (resultado igualmente válido e honesto de reportar).
- Preste atenção especial aos casos 7 e 8 (loop) — testam diretamente a hipótese de que a divergência vem da profundidade de unwind necessária.

## Nota sobre test_gps_cbmc_fair.cpp
Este arquivo é a versão CBMC do GPS com janela justa (len >= 190), equivalente ao
sub-teste ESBMC que produziu o contraexemplo original. Roda com:
```bash
cbmc test_gps_cbmc_fair.cpp --unwind 10 --pointer-check --bounds-check --signed-overflow-check --unsigned-overflow-check
```
