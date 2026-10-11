# Verificação do GPS uBlox — patch (PR #27576) e driver de produção

Pasta criada em 11/10/2026. Itens 4.1 e 4.2 do plano da dissertação.

**Os tempos registrados em `logs/nuvem/` não são comparáveis** aos da Tabela II
(medida no iartes). A máquina da nuvem tem 4 núcleos, e a compilação do ESBMC
rodou em paralelo. Desses logs, use só o **veredito**. Tempos válidos virão de
`logs/iartes/`.

## Por que esta pasta existe

O artigo (Seção V-D, "Patch verification") registra duas lacunas:

1. O `0 of 7 failed` do CBMC no harness com patch (`cbmc/resultado_patch_gps.txt`)
   não isola o efeito da guarda. O harness sem patch dá o mesmo `0 of 7`, porque os
   dois assumem `dump_buffer.len < GPS_DUMP_DATA_SIZE`, o que exclui justamente os
   estados que a guarda trata.
2. Não há log do ESBMC para o harness com patch.

Além disso, o mantenedor do PX4 (revisão do PR #27576, 3/7/2026) argumentou
que `len > sizeof(data)` é inalcançável no driver real. O artigo aceita isso por
análise de código. Aqui o argumento é verificado formalmente.

## Arquivos

| Arquivo | O que é |
|---|---|
| `prod_common.h` | Cópia literal do laço de `GPS::dumpGpsData()` (PX4 `8be2590c15`, `src/drivers/gps/gps.cpp` 727–761), com os tipos reais: `size_t len`, `sizeof(dump_data->data) = 79` (`msg/GpsDump.msg`), `uint8_t dump_data->len` |
| `test_gps_prod_inducao.cpp` | **4.2.** Passo indutivo do invariante `len < 79`: a partir de qualquer estado que o satisfaz, uma iteração do laço não sai do buffer e preserva o invariante |
| `test_gps_prod_sem_invariante.cpp` | **4.2, controle negativo.** Mesmo corpo, sem a hipótese. Tem que falhar |
| `test_gps_fair_sem_patch.cpp` | **4.1.** Harness de janela justa (`len >= 190`) sem patch; mesma lógica de `seeded_tests/test_gps_cbmc_fair.cpp` |
| `test_gps_patch_fair.cpp` | **4.1.** Mesmo `main`, com a guarda do PR #27576 (forma de `cbmc/test_gps_patch.cpp`: `int available`, zera `len` e interrompe) |
| `test_gps_patch_v2.cpp` | **4.1.** Mesmo `main`, com a guarda na forma pedida pelo mantenedor (zera `len` no topo do laço e segue) |
| `test_gps_driver_patch_pr.cpp`, `test_gps_driver_patch_v2.cpp` | **4.1, ESBMC.** Cópias de `resultados/GPS/test_gps_driver.cpp` (harness da Tabela II), alteradas **só** no laço de `dumpGpsData()`. Diferença: `diff --strip-trailing-cr`. As linhas do traço deslocam +2 (cabeçalho) |
| `rodar_verificacao_gps.sh` | Roda tudo e grava os logs com cabeçalho (data, máquina, CPU, versão, comando) e o tempo de parede |

## Base da indução (4.2)

- **Base:** `GPS::initializeCommunicationDump()` faz `memset(..., 0)` nos dois
  buffers (`gps.cpp` 717–718), então `len = 0 < 79`.
- **Passo:** `test_gps_prod_inducao.cpp`.
- **Premissa:** `dumpGpsData()` é o único escritor de `dump_data->len`. Conferido
  com `git grep` no commit `8be2590c15`: em `gps.cpp`, as escritas estão nas linhas
  747–757, e os ponteiros `_dump_to_device`/`_dump_from_device` são membros privados.

Com base e passo, o invariante vale para qualquer número de iterações e chamadas,
sem limite de desenrolamento. O `memcpy` fica dentro dos limites porque é checado
no passo.

## Resultados (nuvem, CBMC 5.95.1, 11/10/2026)

| Harness | Configuração | Veredito | Propriedades |
|---|---|---|---|
| `test_gps_prod_inducao` | `--pointer-check --bounds-check` | SUCCESSFUL | 0 de 124 |
| `test_gps_prod_sem_invariante` | idem | **FAILED** (`memcpy destination region writeable`; `dump.len=128`, `len=77`) | 1 de 123 |
| `test_gps_fair_sem_patch` | unwind 2 (flags de `cbmc/seeded_gps/COMANDO.txt`) | **FAILED** (`memcpy destination region writeable`) | 1 de 73 |
| `test_gps_patch_fair` | unwind 2 | SUCCESSFUL | 0 de 73 |
| `test_gps_patch_v2` | unwind 2 | SUCCESSFUL | 0 de 79 |

Unwind 3 e 5 e o ESBMC 8.1 + Boolector 3.2.4: em andamento. Esta tabela vai ser
atualizada.

## Como rodar no iartes

```
./rodar_verificacao_gps.sh iartes ~/esbmc-v8.1/bin/esbmc cbmc 1800 3
```
