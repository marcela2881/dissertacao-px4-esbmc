# Rascunho das issues para o PX4 — v4 (corrigido)

Marcela — revisão de 08/10/2026 (substitui o rascunho de 19/08/2026)

> **Antes de colar qualquer texto no GitHub:** (1) substitua `<DOI-NOVO>` pelo DOI da nova tag do Zenodo (depois de subir medianas_logs, replays e logs de sanitizadores); (2) cole a saída *verbatim* dos comandos, rodada no `iartes`; (3) confirme com o Prof. Lucas a regra sobre menção a IA (ver `5_submissao_px4/guia_submissao.md`). Todos os testes foram feitos no PX4 v1.16, commit `8be2590c15`.

Professor, segue como eu redigiria cada uma. A diferença em relação ao que fiz
no PR da temperatura é que aqui eu **não proponho a correção**: relato o que o
verificador encontrou, digo até onde a evidência vai, e pergunto aos
mantenedores se o estado é atingível na prática. Se eles decidirem que vale
corrigir, aí sim eu abro o PR — e com a solução que eles apontarem.

Padrão que segui em todas:

- título descreve o que foi encontrado, sem adjetivo de gravidade
- o código real vem citado com arquivo e linha
- reprodução completa, com harness e comando
- uma seção separando o que é alcançável no modelo do que eu não estabeleci
- pergunta direta aos mantenedores, em vez de patch pronto

Não abri issue para o `logger.cpp` (sem violação de memória nos comprimentos modelados) nem para o `Common.cpp` (achado **retirado**, ver Issue 2).

Em todas, o corpo deve dizer explicitamente o que o harness **assume** e o que o verificador **deriva** da entrada.

---

## Issue 1 — ft7_technologies.cpp

**Título:** `Unbounded byte counters in FT Technologies wind sensor driver can write past five buffers`

```markdown
### Summary

The FT Technologies wind sensor driver keeps byte counters that persist across
calls to `collect()` and are only reset when a `\n` is found in the serial
stream. If the sensor sends malformed data without a line terminator, the
counters keep growing across calls and index past the end of five separate
buffers.

### Affected code

`src/drivers/wind_sensor/ft_technologies/ft7_technologies.cpp`

The counters (`_byte_counter` and four similar ones) are member variables. They
are incremented as bytes arrive and reset only inside the branch that handles
the line terminator. A stream that never delivers `\n` never triggers the reset.

### How it was found

Bounded model checking with ESBMC v8.1.0 on a hand-written harness that isolates
the parsing function and leaves the serial input non-deterministic. The result
was cross-checked with CBMC 5.95.1 and replayed with AddressSanitizer against
the uninstrumented parser.

Harness, replay and logs:
https://github.com/marcela2881/dissertacao-px4-esbmc (Zenodo: <DOI-NOVO>)

```
esbmc test_ft7_technologies.cpp --incremental-bmc --overflow-check --timeout 120
cbmc test_ft7_technologies.cpp --unwind 25 --bounds-check \
     --signed-overflow-check --unsigned-overflow-check
g++ -fsanitize=address -g -o r3 replay_ft7.cpp && ./r3
```

All three report the out-of-bounds access. ESBMC reaches it at k=20.

### Scope of the finding

The violation is reachable in the model. I did not establish whether a physical
FT742 sensor, or a fault on the UART line, can produce a stream long enough
without a terminator to reach the state — that depends on call sequences from
the scheduler that my harness abstracts away.

### Question for maintainers

Is there a guarantee upstream of `collect()` that bounds how much data can
arrive between line terminators? If there is, this is a non-issue and I would
appreciate knowing where it is enforced, so I can correct my model.
```

---

## Issue 2 — Common.cpp — **RETIRADA / NÃO ABRIR**

**Motivo (reexecução de 08/10/2026, CBMC 5.95.1, ver `4_resultados_ATUAL/cand1_reexecucao_claude/`):**

O harness original assumia `_next_buffer_idx` em `[0, EVENT_BUFFER_SIZE + 4]`. Isso
inclui estados em que o índice já está *acima* do tamanho do buffer — mas o próprio
guard (`if (total_size > sizeof(_event_buffer) - _next_buffer_idx) return false;`)
mantém, por indução, `_next_buffer_idx <= sizeof(_event_buffer)`. O contraexemplo
`_next_buffer_idx = 74` (buffer de 72) só existe porque o harness assume um estado
que o código não produz.

| Variante | `EVENT_BUFFER_SIZE` | Premissa sobre o índice | CBMC 5.95.1 |
|---|---|---|---|
| orig_72  | 72  | `0 <= idx <= SIZE+4` | 5 de 10 falharam |
| inv_72   | 72  | `0 <= idx <= SIZE`   | **0 de 10 falharam** (SUCCESSFUL) |
| orig_168 | 168 | `0 <= idx <= SIZE+4` | 5 de 10 falharam |
| inv_168  | 168 | `0 <= idx <= SIZE`   | **0 de 10 falharam** (SUCCESSFUL) |

Ou seja: com a invariante realista, não há violação em nenhum dos dois tamanhos. A
falha com `SIZE+4` é consequência da premissa, não do código. (Observação: o
`ORB_QUEUE_LENGTH = 16` do `Event.msg` no fork é outro fator a conferir, mas não
altera a conclusão acima.) Esta reexecução foi feita no ambiente da Claude, não no
`iartes`; **repetir no `iartes` antes de citar em qualquer lugar.**

Texto para o artigo/dissertação: tratar como "achado inicial invalidado por
premissa do harness" — exemplo de falso positivo causado por assunção de estado.
Não abrir issue.

---

## Issue 3 — externalChecks.cpp

**Título:** `num_events from external component indexes fixed-size array without bounds check`

```markdown
### Summary

`num_events` arrives from an external component over uORB, is typed `uint8_t`
(so it can hold 0–255), and is used as a loop bound indexing `events[5]` — an
array with five elements. There is no check between reading the field and using
it.

### Affected code

`src/modules/commander/HealthAndArmingChecks/checks/externalChecks.cpp`

### How it was found

ESBMC v8.1.0. The verifier reports a `memcpy` whose size it represents internally as `SIZE_MAX`
(2^64−1). That value is ESBMC's marker for an unbounded/invalid region, not a size
the program computes; the actual defect is the unchecked `num_events` bound. CBMC 5.95.1
and AddressSanitizer confirm.

```
esbmc test_external_checks_events.cpp --incremental-bmc --overflow-check --timeout 120
cbmc test_external_checks_events.cpp --unwind 10 --bounds-check \
     --signed-overflow-check --unsigned-overflow-check
g++ -fsanitize=address -g -o r2 replay_external_checks.cpp && ./r2
```

ESBMC reaches the violation at k=6.

### Scope of the finding

This one differs from the others in one respect: the value comes from outside
the flight stack, over uORB, so it is not constrained by internal invariants of
the module. That said, I did not verify whether any publisher in the tree can
actually emit `num_events > 5`, nor whether the middleware validates it.

### Question for maintainers

Is `num_events` validated anywhere between the publisher and this consumer? If
the contract is that publishers must respect the array size, is that documented
somewhere I could reference?
```

---

## Issue 4 — bst.cpp

**Título:** `dev_name_len read from I2C device indexes dev_name[32] without validation`

```markdown
### Summary

`dev_name_len` is read from an external I2C device and used directly to index
`dev_name[32]`, with no check that it is within bounds.

### Affected code

`src/drivers/telemetry/bst/bst.cpp`, line 208:

```c
dev_info_reply.payload.dev_name[dev_info_reply.payload.dev_name_len] = '\0';
```

`dev_name_len` is `uint8_t` (0–255) and comes from the device response, so any
value in that range reaches the index.

### How it was found

ESBMC v8.1.0 flagged it at k=1. To rule out a harness artifact, I replayed it
against a copy of `probe()` and `crc8()` taken verbatim from the driver, with
only `I2C::transfer()` replaced by a stub that returns `dev_name_len = 244` —
the exact counterexample value.

```
esbmc test_bst_dev_name.cpp --incremental-bmc --overflow-check --timeout 120
cbmc test_bst_dev_name.cpp --unwind 10 --bounds-check \
     --signed-overflow-check --unsigned-overflow-check
g++ -fsanitize=address -g -O0 -o r7 replay_bst_probe_only.cpp && ./r7
```

AddressSanitizer, running the verbatim `probe()` (x86_64 Linux, g++): `stack-buffer-overflow`, **READ of size 34**, frame `bst_probe_real` (`replay_bst_probe_only.cpp:136`), object `dev_info_reply` at `[48, 90)`, access at offset 90. <paste verbatim output here>

The same replay was run independently on macOS/aarch64 with Apple clang, giving
the same object bounds and frame, with READ of size 37 (the size differs because of
platform stack layout; the violation is the same).

Note on the ASan output: it flags the read performed by `%s` in the `PX4_DEBUG`
call, not the write itself. The write at `dev_name[244]` lands far past the object and ASan does not
report that store directly (it may skip over the redzone; this was not
verified here). Compiling with `-fsanitize=local-bounds` should pin the write;
this is still to be run.

### Scope of the finding

Reachable in the model; the replay shows the verbatim `probe()` executing with
the stub returning 244, and ASan reports the out-of-bounds access on the stack
object (the `dev_info_reply` is a stack object here, not heap). What I did not establish is
whether a real BST device — or a faulty one, or noise on the I2C line — can
deliver that value in practice.

### Question for maintainers

Is there an expectation that the BST device always returns `dev_name_len <= 32`?
If the driver is meant to trust the device here, that would be useful to know.
```

---

## Issue 5 — MicroStrain.cpp

**Título:** `Off-by-one when null-terminating _port in MicroStrain constructor`

```markdown
### Summary

In the constructor, `max_len` can equal `sizeof(_port)`, and the null terminator
is then written at `_port[max_len]` — one past the end of the array.

### Affected code

`src/drivers/ins/microstrain/MicroStrain.cpp`, line 55:

```c
const size_t max_len = math::min(sizeof(_port), strnlen(uart_port, sizeof(_port)));
memcpy(_port, uart_port, max_len);
_port[max_len] = '\0';
```

`_port` has 128 elements, so valid indices are 0–127. If `uart_port` has 128 or
more characters with no null in the first 128, `strnlen` returns 128, `min`
returns 128, and the write lands at `_port[128]`.

The `memcpy` itself is fine — copying 128 bytes into a 128-byte array is legal.
Only the terminator is out of bounds.

### How it was found

ESBMC v8.1.0 with `uart_port` left fully non-deterministic and `strnlen`/`min`
computed rather than assumed, so the critical state (`max_len = 128`) is *derived by the verifier from the
input* rather than assumed by the harness. (The earlier v2 harness assigned
`max_len = PORT_SIZE` directly and is superseded.)

```
esbmc test_microstrain_port_v3.cpp --unwind 200 --overflow-check --timeout 300
cbmc test_microstrain_port_v3.cpp --unwind 200 --bounds-check \
     --signed-overflow-check --unsigned-overflow-check
g++ -fsanitize=address -g -O0 -o r8 replay_microstrain_v3.cpp && ./r8
```

Counterexample: 128 non-null bytes followed by a terminator at position 128.
AddressSanitizer on the replay:

```
WRITE of size 1
0 bytes after 128-byte region
heap-buffer-overflow
```

### Scope of the finding

Reachable in the model given a sufficiently long port string. `uart_port` comes
from configuration, so whether a 128-character device path can be supplied in
practice depends on how the parameter is set and whether anything upstream
limits its length. I did not check that.

### Question for maintainers

Is the port string length bounded anywhere before it reaches this constructor?
```

---

## Issue 6 — IridiumSBD.cpp

**Título:** `uint16_t underflow in _packet_length when buflen exceeds remaining packet`

```markdown
### Summary

In `write()`, the space check compares the remaining TX space against
`_packet_length` (a value around 7 in the model) instead of against `buflen`, the
number of bytes actually copied. `_packet_length` is `uint16_t` and is then
decremented by `buflen`, so it can wrap. The result is a `memcpy` of `buflen`
bytes at `_tx_buf + _tx_buf_write_idx` that can run past the 340-byte buffer.

### Affected code

`src/drivers/telemetry/iridiumsbd/IridiumSBD.cpp`

### How it was found

ESBMC v8.1.0 reports the violation at k=1. CBMC 5.95.1 confirms, with two
failing properties:

```
[iridiumsbd_write.precondition_instance.3] line 130
    memcpy destination region writeable: FAILURE
[main.assertion.1] line 155
    assertion _tx_buf_write_idx <= SATCOM_TX_BUF_LEN: FAILURE
** 2 of 10 failed
```

The violating range of `second_buflen` is 331–340 (ESBMC 8.1.0 picked 335; the
ASan replay used 334; 331 is the minimum). The cause is that the guard
`SATCOM_TX_BUF_LEN - _tx_buf_write_idx - _packet_length < 0` does not account for
`buflen`.

```
esbmc test_iridiumsbd_write.cpp --incremental-bmc --overflow-check \
      --no-library --timeout 120
cbmc test_iridiumsbd_write.cpp --unwind 400 -D'ssize_t=long' --bounds-check \
     --signed-overflow-check --unsigned-overflow-check
g++ -fsanitize=address -g -o r6 replay_iridiumsbd.cpp && ./r6
```

(`-D'ssize_t=long'` is needed **only** for CBMC 5.95.1, which does not recognise
`ssize_t`; on CBMC 6.x/macOS it should be omitted.)

### Scope of the finding

Reachable in the model for the range above. Whether a caller can pass such a `buflen` depends on the sequence of `write()` calls, which I
did not model exhaustively.

### Question for maintainers

Is there an invariant that callers must respect regarding `buflen` and the
current packet state? If so, is it enforced or documented anywhere?
```

---

## Issue 7 — sumd.cpp

**Título:** `Off-by-one in SUMD decoder loop condition writes one byte past sumd_data[64]`

```markdown
### Summary

The loop condition uses `<=` where `<` appears to be intended, allowing a write
at `sumd_data[64]` in an array with 64 elements.

### Affected code

`src/lib/rc/sumd.cpp`

```c
_rxlen <= (_rxpacket.length * 2)
```

With `length = 32`, which is a legitimate maximum, `_rxlen` reaches 64 and the
write lands one past the end (line 184 in the delivered file; the ESBMC log reports 162 — confirm the line against the commit before posting).

### Practical impact is limited

I want to be explicit about this: the write stays inside the same allocated
object. `sumd_data` sits at offset 3 of `ReceiverFcPacketHoTT`, which is 71
bytes, so index 64 lands on byte 67 — the `crc16_high` field. It is an
out-of-bounds array access by the language rules, and it corrupts a CRC field,
but it is not a write past the end of the allocation.

This is also why AddressSanitizer does not report it: ASan instruments
allocation boundaries, not member boundaries, so intra-object overflow is
invisible to it. UndefinedBehaviorSanitizer does catch it:

```
replay_sumd.cpp:37:29: runtime error: index 64 out of bounds
                       for type 'unsigned char [64]'
```

### How it was found

```
esbmc test_sumd_decode.cpp --unwind 70 --overflow-check
cbmc test_sumd_decode.cpp --unwind 70 --bounds-check \
     --signed-overflow-check --unsigned-overflow-check
g++ -fsanitize=undefined -fno-sanitize-recover=all -g -O0 -o r4 replay_sumd.cpp && ./r4
```

ESBMC: `_rxlen = 64`, array bounds violated at `test_sumd_decode.cpp:162`.

(With `--incremental-bmc` the k-induction diverges on the nested CRC16 loop and
times out. The loops have a known static bound — 65 decode calls × 8 CRC
iterations — so an explicit unwind is justified here.)

### Scope of the finding

Reachable in the model with a legitimate packet length. Given that the
corruption lands on a CRC field within the same object, I am reporting this as a
correctness issue rather than a memory-safety one, and I would understand if it
is judged low priority.
```

---

## Uma observação sobre a ordem

Se o senhor concordar, eu abriria uma por vez em vez de todas de uma vez.
Sete issues do mesmo autor no mesmo dia pode parecer despejo automatizado, e
depois do episódio da temperatura prefiro ir devagar. Começando pelo
`ft7_technologies.cpp` dá pra ver como a comunidade reage ao formato antes de
mandar as outras.
