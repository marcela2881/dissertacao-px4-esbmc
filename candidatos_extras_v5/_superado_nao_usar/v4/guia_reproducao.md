# Como reproduzir os testes — 8 candidatos PX4 (v4)

PX4 v1.16, commit `8be2590c15`.

## Ambiente (único)
- Referência dos logs: servidor `iartes`, Ubuntu 24.04 — ESBMC v8.1.0 (Boolector padrão), CBMC 5.95.1, g++ com ASan/UBSan
- Reprodução independente do orientador: macOS/arm64, ESBMC 8.4.0, CBMC 6.8.0
- O flag `-D'ssize_t=long'` é necessário **somente** no CBMC 5.95.1; em 6.x/macOS ele atrapalha — omitir
- Caminhos: `1_harnesses/ANTIGO/rodada1|rodada2` e `1_harnesses/ATUAL`; replays em `2_replays/...`; logs em `3_resultados_ANTIGO/...` e `4_resultados_ATUAL/` (nomes com sufixo `_v3`/`_probe_only` etc.)

---

## O que é cada candidato

### Candidato 1 — Common.cpp
**Arquivo PX4:** `src/modules/commander/HealthAndArmingChecks/Common.cpp`
**Bug:** Mistura de tipo sinalizado (`int _next_buffer_idx`) e não-sinalizado (`size_t`) na guarda de tamanho do buffer de eventos. Se `_next_buffer_idx` ultrapassar o tamanho do buffer, a subtração `sizeof(_event_buffer) - _next_buffer_idx` estoura para um número gigante, fazendo a guarda não disparar.
**Harness:** `test_common_event_buffer.cpp`
**STATUS v4: RETIRADO.** O harness assume `idx` em `[0, SIZE+4]`, mas o guard mantém `idx <= SIZE`. Com `idx <= SIZE`, o CBMC 5.95.1 dá 0 de 10 falhas (SIZE 72 e 168). Ver `4_resultados_ATUAL/cand1_reexecucao_claude/`. **Não usar como achado.**

### Candidato 2 — externalChecks.cpp
**Arquivo PX4:** `src/modules/commander/HealthAndArmingChecks/checks/externalChecks.cpp`
**Bug:** Campo `num_events` (tipo `uint8_t`, range 0-255) vem de componente externo via uORB e é usado como limite de loop indexando `events[5]` (array fixo de 5 posições) sem nenhuma checagem.
**Harness:** `test_external_checks_events.cpp`

### Candidato 3 — ft7_technologies.cpp
**Arquivo PX4:** `src/drivers/wind_sensor/ft_technologies/ft7_technologies.cpp`
**Bug:** Contador `_byte_counter` (e outros 4 contadores similares) persiste entre chamadas de `collect()` e só reseta quando encontra `\n` na linha serial. Se dados malformados chegarem sem `\n`, o contador cresce indefinidamente e escreve além dos limites de 5 buffers diferentes.
**Harness:** `test_ft7_technologies.cpp`

### Candidato 4 — sumd.cpp
**Arquivo PX4:** `src/lib/rc/sumd.cpp`
**Bug:** Off-by-one na condição do loop: `_rxlen <= (_rxpacket.length * 2)` deveria ser `<`. Com `length=32` (valor máximo legítimo), permite escrita em `sumd_data[64]` num array de 64 posições (índices válidos 0-63).
**Harness:** `test_sumd_decode.cpp`
**Ressalva:** a escrita cai dentro da mesma struct/union — severidade prática baixa.
**Nota sobre sanitizadores:** o AddressSanitizer **não detecta** este caso. A
escrita em `sumd_data[64]` permanece dentro do mesmo objeto `malloc`
(`ReceiverFcPacketHoTT`, 71 bytes; `sumd_data` no offset 3, logo o índice 64
atinge o byte 67, campo `crc16_high`). O ASan instrumenta as bordas da
alocação, não os limites entre membros. O UBSan confirma, porque o subscrito
incide sobre um array de tamanho estático.

### Candidato 5 — logger.cpp
**Arquivo PX4:** `src/modules/logger/logger.cpp`
**Hipótese:** Truncamento de `uint8_t key_len` recebendo retorno de `snprintf()`.
**Resultado:** **sem violação de memória para os comprimentos modelados e qualquer `key_len` 0–255.** O SUCCESSFUL em k=300 vale para o harness (laço literal de 299 iterações; `snprintf` modelado devolve valor irrestrito para `%s` simbólico); **não** é prova de ausência de bug no `logger.cpp` real.
**Harness:** `test_logger_write_info_multiple.cpp`
**Nota de execução:** o laço `for (i = 0; i < 299; i++)` do harness excede o
limite default de k-step (50) do `--incremental-bmc`, e o ESBMC devolve
`VERIFICATION UNKNOWN`. Com `--max-k-step 320` o resultado é
`VERIFICATION SUCCESSFUL`, e a condição de avanço converge:
`Solution found by the forward condition; all states are reachable (k = 300)`.


### Candidato 6 — IridiumSBD.cpp
**Arquivo PX4:** `src/drivers/telemetry/iridiumsbd/IridiumSBD.cpp`
**Bug:** a guarda `SATCOM_TX_BUF_LEN - _tx_buf_write_idx - _packet_length < 0` compara com `_packet_length` (≈7) e **não** com `buflen`. Depois `_packet_length -= (uint16_t)buflen` (pode dar wrap) e `memcpy(_tx_buf + _tx_buf_write_idx, buffer, buflen)` ultrapassa o buffer de 340 bytes. Faixa violadora de `second_buflen`: 331–340 (ESBMC escolheu 335; replay 334; mínimo 331).
**Harness:** `test_iridiumsbd_write.cpp`
**Nota sobre o CBMC:** o CBMC 5.95.1 não reconhece `ssize_t` e aborta com
`CONVERSION ERROR` antes de verificar. É necessário `-D'ssize_t=long'`. Com
essa definição e `--unwind 400`, o 5.95.1 decide e reporta FAILED nas mesmas
duas propriedades que o CBMC 6.8.0 (linhas 130 e 155), o que indica que o
fator determinante é o unwind, não a versão da ferramenta.

### Candidato 7 — bst.cpp
**Arquivo PX4:** `src/drivers/telemetry/bst/bst.cpp`
**Bug:** `dev_name_len` (tipo `uint8_t`, 0-255) vem do dispositivo I2C externo e é usado para indexar `dev_name[32]` sem checagem. Linha 208: `dev_info_reply.payload.dev_name[dev_info_reply.payload.dev_name_len] = '\0';`
**Harness:** `test_bst_dev_name.cpp`
**Replay real:** `replay_bst_probe_only.cpp` contém `probe()` e `crc8()` verbatim do PX4. Só `I2C::transfer()` foi substituído por stub que injeta `dev_name_len=244` (contraexemplo exato do ESBMC).
**Ressalva sobre o replay anterior:** o `replay_bst_real.cpp` (mantido no pacote para
rastreabilidade) **não alcança** `bst_probe_real()`. Seu `main()` injeta o contraexemplo numa
cópia heap-alocada na linha 172 antes de chamar `bst_probe_real()` na linha 179; a escrita da
linha 172 dispara o ASan e aborta o processo. O log `asan_bst_codigo_real.txt` termina em
`replay_bst_real.cpp:172 in main`, confirmando que o `probe()` verbatim nunca executou — o
overflow ali registrado é o da struct sintética do `main`. O `replay_bst_probe_only.cpp`
preserva todo o arquivo original até `main()` e troca apenas o driver por um que chama
somente `bst_probe_real()`.

### Candidato 8 — MicroStrain.cpp
**Arquivo PX4:** `src/drivers/ins/microstrain/MicroStrain.cpp`
**Bug:** Off-by-one no construtor (linha 55). `_port[128]`, índices válidos 0-127. O código calcula `max_len = min(sizeof(_port), strnlen(uart_port, sizeof(_port)))`. Se `uart_port` tiver 128+ caracteres sem terminador nos primeiros 128, `max_len=128` e `_port[128] = '\0'` escreve além do array.
**Harness:** `test_microstrain_port_v3.cpp`
**Contraexemplo:** `uart_port[0..127]` não-nulos, `uart_port[128] = '\0'` → `strnlen` devolve 128 → `max_len = 128`.

**Sobre o harness v2 (substituído).** A versão anterior tinha dois problemas.
O primeiro, apontado pelo orientador: a asserção do `main()` era
`assert(PORT_SIZE <= PORT_SIZE - 1)`, isto é `128 <= 127`, uma contradição.
Ela nunca chegou a ser alcançada — o ESBMC para na primeira violação, que
ocorria antes, na verificação automática de limites da linha 38 — mas era
ruído e saiu.

O segundo problema é mais sério e foi o que motivou a reescrita: o v2 fixava
`const size_t max_len = PORT_SIZE;` por atribuição direta, sem `uart_port`,
sem `strnlen` e sem `min`. Com isso o verificador demonstrava apenas que
`_port[128]` estoura um array de 128 posições, o que é aritmética e não
verificação. A pergunta relevante — existe entrada que leve o driver a
`max_len = 128`? — não era feita.

O v3 deixa `uart_port` não-determinístico byte a byte e calcula `strnlen` e
`min` como no driver real. O contraexemplo passa a ser derivado da entrada,
não assumido.

---

## Comandos para reproduzir

### Passo 1 — Candidatos 1 a 5 (pasta `rodada1/`)

```bash
cd 1_harnesses/ANTIGO/rodada1/   # harnesses; replays em 2_replays/ANTIGO/rodada1/

# ESBMC
esbmc test_common_event_buffer.cpp --incremental-bmc --overflow-check --timeout 120
esbmc test_external_checks_events.cpp --incremental-bmc --overflow-check --timeout 120
esbmc test_ft7_technologies.cpp --incremental-bmc --overflow-check --timeout 120

# sumd: --incremental-bmc diverge no laco CRC16 aninhado e estoura o timeout.
# Todos os lacos tem cota estatica conhecida (65 chamadas de decode x 8 iteracoes
# de CRC), entao o unwind explicito e justificado e fecha o caso.
esbmc test_sumd_decode.cpp --unwind 70 --overflow-check

# logger: --max-k-step 320 e necessario para cobrir as 299 iteracoes do laco.
# Sem ele o ESBMC devolve UNKNOWN em vez de SUCCESSFUL.
esbmc test_logger_write_info_multiple.cpp --incremental-bmc --max-k-step 320 --overflow-check --timeout 600

# CBMC
cbmc test_common_event_buffer.cpp --unwind 10 --bounds-check --signed-overflow-check --unsigned-overflow-check
cbmc test_external_checks_events.cpp --unwind 10 --bounds-check --signed-overflow-check --unsigned-overflow-check
cbmc test_ft7_technologies.cpp --unwind 25 --bounds-check --signed-overflow-check --unsigned-overflow-check
cbmc test_sumd_decode.cpp --unwind 70 --bounds-check --signed-overflow-check --unsigned-overflow-check

# AddressSanitizer
g++ -fsanitize=address -g -o r1 replay_common_event.cpp && ./r1
g++ -fsanitize=address -g -o r2 replay_external_checks.cpp && ./r2
g++ -fsanitize=address -g -o r3 replay_ft7.cpp && ./r3

# sumd: o ASan NAO detecta este caso (overflow intra-objeto). Use UBSan.
g++ -fsanitize=undefined -fno-sanitize-recover=all -g -O0 -o r4 replay_sumd.cpp && ./r4
```

### Passo 2 — Candidatos 6 a 8 (pasta `rodada2/`)

```bash
cd 1_harnesses/ANTIGO/rodada2/   # bst/iridium; microstrain v3 em 1_harnesses/ATUAL/; replays em 2_replays/

# ESBMC
esbmc test_bst_dev_name.cpp --incremental-bmc --overflow-check --timeout 120
esbmc test_iridiumsbd_write.cpp --incremental-bmc --overflow-check --no-library --timeout 120
esbmc test_microstrain_port_v3.cpp --unwind 200 --overflow-check --timeout 300

# CBMC
cbmc test_bst_dev_name.cpp --unwind 10 --bounds-check --signed-overflow-check --unsigned-overflow-check
cbmc test_microstrain_port_v3.cpp --unwind 200 --bounds-check --signed-overflow-check --unsigned-overflow-check

# IridiumSBD: -D'ssize_t=long' SO no CBMC 5.95.1 (nao reconhece ssize_t -> CONVERSION ERROR).
# Em CBMC 6.x/macOS, REMOVER o -D. Esta e a unica linha de CBMC para este candidato.
# Ressalva: se o CBMC reportar 'unwinding assertion' falha, aumente --unwind (nao use --unwinding-assertions para concluir SUCCESS sem checar).
cbmc test_iridiumsbd_write.cpp --unwind 400 -D'ssize_t=long' --bounds-check --signed-overflow-check --unsigned-overflow-check

# AddressSanitizer
g++ -fsanitize=address -g -o r6 replay_iridiumsbd.cpp && ./r6
# bst: use o replay que realmente alcanca probe(). O replay_bst_real.cpp
# aborta no main antes de chamar bst_probe_real() — ver Candidato 7 acima.
g++ -fsanitize=address -g -o r7 replay_bst_probe_only.cpp && ./r7
g++ -fsanitize=address -g -O0 -o r8 replay_microstrain_v3.cpp && ./r8
```

---

## O que esperar

| # | Módulo | ESBMC | CBMC | ASan |
|---|---|---|---|---|
| 1 | Common.cpp | FAILED (premissa do harness) | FAILED (idem); **0 de 10 com idx<=SIZE** | **RETIRADO** |
| 2 | externalChecks.cpp | FAILED* | FAILED | overflow |
| 3 | ft7_technologies.cpp | FAILED | FAILED | overflow |
| 4 | sumd.cpp | FAILED** | FAILED | ASan não detecta; UBSan confirma**** |
| 5 | logger.cpp | SUCCESSFUL***** | — | — |
| 6 | IridiumSBD.cpp | FAILED | FAILED*** | overflow |
| 7 | bst.cpp | FAILED (k=1) | FAILED | overflow (READ 34 Linux / 37 macOS) |
| 8 | MicroStrain.cpp (**harness v3**) | FAILED | FAILED | heap-buffer-overflow, WRITE of size 1 |

(Linha 8: a v2 do harness era inválida por assumir `max_len`; o resultado **v3** é válido, com entrada não-determinística.)

*externalChecks ESBMC pode dar timeout dependendo do solver — CBMC e ASan confirmam.

**sumd: com `--incremental-bmc` o ESBMC diverge no loop CRC aninhado e estoura o timeout.
Com `--unwind 70` fecha em FAILED (`_rxlen = 64` → array bounds violated,
`test_sumd_decode.cpp:162`). Confirmado pelas três ferramentas (ASan não; UBSan sim).

***IridiumSBD: com `--unwind 400`, suficiente para o memcpy de 340 bytes, o CBMC decide e
reporta FAILED (`memcpy destination region writeable` na linha 130 e a asserção
`_tx_buf_write_idx <= SATCOM_TX_BUF_LEN` na linha 155).

****sumd: o AddressSanitizer **não detecta** este caso — a escrita é intra-objeto e o ASan
instrumenta apenas as bordas da alocação. O UBSan confirma
(`index 64 out of bounds for type 'uint8_t [64]'`).

*****logger: requer `--max-k-step 320`. SUCCESSFUL vale para o harness (laço de 299 iterações, `snprintf` modelado); não generalizar. Com o default (50) o ESBMC devolve UNKNOWN.

*Nota MicroStrain:* resultado da tabela refere-se ao harness **v3**; o v2 está superado (ver Candidato 8).
