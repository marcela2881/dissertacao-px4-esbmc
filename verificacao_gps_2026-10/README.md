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
| `logs/nuvem/validacao_build/` | Logs da validação do build do ESBMC contra julho |
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

## Resultados (nuvem, 11/10/2026: ESBMC 8.1.0 + Boolector 3.2.4 e CBMC 5.95.1)

O ESBMC foi compilado a partir da tag `v8.1` com Boolector 3.2.4, porque o
binário de release não estava acessível pela nuvem. **Validação do build:** rodando
os harnesses de julho, ele reproduz exatamente os logs de `medianas_logs/`:
Barometer 93, IMU 120 (`count = 9088`), Navigator 50, PWM 38 e GPS 4463 VCCs
(`input_len = 10`, `.len = 207`). Logs em `logs/nuvem/validacao_build/` e
`logs/nuvem/t2_test_gps_driver_esbmc_u10_r1.log`.

### 4.1: patch, mesmas condições com e sem patch

| Ferramenta / harness | Sem patch | Patch do PR #27576 | Patch v2 (mantenedor) |
|---|---|---|---|
| ESBMC, harness da Tabela II, flags da Tabela II (`--unwind 10 --no-unwinding-assertions --overflow-check`) | **FAILED** (4463 VCCs; 10/207, igual a julho) | SUCCESSFUL (3453 VCCs) | SUCCESSFUL (4760 VCCs) |
| CBMC, janela justa `len >= 190`, unwind 2 | **FAILED** (1 de 73) | SUCCESSFUL (0 de 73) | SUCCESSFUL (0 de 79) |
| CBMC, idem, unwind 3 | **FAILED** (1 de 73) | SUCCESSFUL (0 de 73) | SUCCESSFUL (0 de 79) |
| CBMC, idem, unwind 5 | **FAILED** (1 de 73) | SUCCESSFUL (0 de 73) | SUCCESSFUL (0 de 79) |

Escopo: são verificações **limitadas** (unwind 10 no ESBMC; 2, 3 e 5 no CBMC; sem
unwinding assertions, como na Tabela II). Mostram que, sob as mesmas
pré-condições, a guarda elimina a violação que o harness sem patch exibe. Não
provam a ausência de outros caminhos além do limite.

### 4.2: driver de produção (79 bytes), prova indutiva

| Harness | CBMC | ESBMC (`--unwind 80`, unwinding assertions **ligadas**) |
|---|---|---|
| `test_gps_prod_inducao` | SUCCESSFUL (0 de 124) | SUCCESSFUL (447 VCCs) |
| `test_gps_prod_sem_invariante` (controle) | **FAILED** (`dump.len=128`) | **FAILED** (`dereference failure: Access to object out of bounds`, `.len=146`) |

Aqui o resultado **não é limitado**. O harness não tem laço próprio, e o único
laço (o modelo de `memcpy` do ESBMC, até 79 bytes) é coberto pelo unwind 80, com
a assertion de desenrolamento ligada. Com a base (`memset`) e a premissa de
escritor único, o invariante `len < 79` vale em toda execução do driver real.
Logo, o estado que viola o harness da Tabela II (`len = 207`) é inalcançável na
produção. É o argumento do mantenedor, agora verificado.

## Como rodar no iartes

```
./rodar_verificacao_gps.sh iartes ~/esbmc-v8.1/bin/esbmc cbmc 1800 3
```
