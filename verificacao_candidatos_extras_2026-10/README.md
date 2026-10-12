# Reexecução dos candidatos extras (pacote v4): 12/10/2026

Executado no **ambiente da nuvem (Claude)**, não no `iartes`, com estas ferramentas:

- ESBMC 8.1.0 + Boolector 3.2.4
- CBMC 5.95.1
- g++ 13.3 (ASan/UBSan) e clang 18.1.3 (`-fsanitize=local-bounds`)

Cada log traz no cabeçalho a data, o host, o comando exato e, na última linha, o código de saída.

Os harnesses e replays usados são os do pacote `TUDO_FINAL_v4`. Eles não foram copiados para cá: os scripts recebem o caminho do pacote como argumento.

Para reproduzir:

```
scripts/rodar_guia_v4.sh       <pacote_v4> <saida>            # guia_reproducao.md, comando a comando
scripts/extras_pendencias_v4.sh <pacote_v4> replays/ <saida>  # itens 1-5 de "O que AINDA precisa ser rodado"
```

## 1. Guia de reprodução (`logs_guia_v4/`)

| # | Módulo | ESBMC | CBMC | Sanitizador | Confere com o guia? |
|---|---|---|---|---|---|
| 2 | externalChecks.cpp | FAILED | FAILED (3 de 7) | ASan heap-buffer-overflow | sim |
| 3 | ft7_technologies.cpp | FAILED | FAILED (6 de 19) | ASan heap-buffer-overflow | sim |
| 4 | sumd.cpp (`--unwind 70`) | FAILED (`_rxlen < 64`) | FAILED (2 de 14) | ASan não detecta; UBSan `index 64 out of bounds` | sim |
| 5 | logger.cpp (`--max-k-step 320`) | SUCCESSFUL (k = 300) | — | — | sim |
| 6 | IridiumSBD.cpp | **PARSING ERROR** (ver abaixo) | FAILED (2 de 10) | ASan global-buffer-overflow, WRITE 334 | ESBMC não |
| 7 | bst.cpp | FAILED | FAILED (2 de 3) | ASan stack-buffer-overflow, READ 34 | sim |
| 8 | MicroStrain v3 | FAILED (`max_len < 128`) | FAILED (2 de 8) | ASan heap-buffer-overflow, WRITE 1 | sim |

**Erro no guia (linha 148):** o comando do ESBMC para o IridiumSBD falha no parser com `unknown type name 'ssize_t'`, porque `--no-library` remove a definição do tipo. O comando do CBMC já tinha `-D'ssize_t=long'`. Com o mesmo `-D` no ESBMC o resultado é **FAILED** (`logs_pendencias/esbmc_c6_iridium_ssize.txt`). É preciso corrigir a linha 148 do guia.

## 2. Pendências do CORRECOES_v4 (`logs_pendencias/`)

| Item | Resultado |
|---|---|
| 1. Candidato 1 (Common.cpp) | Com `idx <= SIZE+4`, os dois verificadores dão FAILED (CBMC 5 de 16; ESBMC 4 violações). Com `idx <= SIZE`, os dois dão SUCCESSFUL, tanto em SIZE 72 quanto em 168. O CBMC rodou com `--unwinding-assertions` e o ESBMC com `--unwind 200`. **A retirada do achado se confirma.** Com `--unwind 10`, o ESBMC acusa só uma *unwinding assertion* dentro do `memset` da biblioteca: limite de desenrolamento, não violação (`cand1_esbmc_inv_*.txt` × `cand1_esbmc_inv_*_u200.txt`). |
| 2. UBSan sumd, log completo | `logs_guia_v4/ubsan_c4_sumd.txt`: `replay_sumd.cpp:37:29: runtime error: index 64 out of bounds for type 'unsigned char [64]'`, `exit: 1` |
| 3. bst + `-fsanitize=local-bounds` | O clang interrompe (SIGILL, `exit: 132`) em `replay_bst_probe_only.cpp:134`, que é a linha da escrita `dev_name[dev_name_len] = '\0'` (backtrace no gdb, no próprio log). O ASan do clang não está instalado neste ambiente; o ASan do g++ já está no guia. **Ver a seção 3: este candidato não vale contra o 8be2590c15.** |
| 3b. sumd + `local-bounds` | Não detecta (`exit: 0`): a escrita é intra-objeto, o mesmo motivo pelo qual o ASan não vê. |
| 4. externalChecks `--generate-testcase` | O ESBMC 8.1 gera `testcase.xml` com uma única entrada `0`, porque o `num_events` fica simbólico (`nondet_symbol`) no contraexemplo. Esse testcase **não serve de replay**. O valor concreto vem do trace do CBMC: `reply.num_events=9` (`cbmc_c2_external_trace.txt`). |
| 5. ft7 replay chamando `collect_call()` | Novo `replays/replay_ft7_collect.cpp`. O laço é copiado do 8be2590c15 (linhas 158–234) e os membros estão na ordem do `.hpp`; ver os detalhes logo abaixo. |

Resultados do novo replay do ft7:

- **UBSan, código original:**
  - cenário 1: `index 30 out of bounds for type 'char [30]'`;
  - cenário 2 (`$A,WVP=` + dígitos): `index 5 out of bounds for type 'char [5]'`.
- **ASan:** não detecta (escrita intra-objeto). No cenário 1, porém, o estouro sobrescreve o membro vizinho: `_interval` passa de 100000 para **88152**.
- **Com as guardas do commit upstream 3acf7935a0 (`-DFIXED`):** nenhum relato, e `_interval` permanece 100000.

O replay antigo (`replay_ft7.cpp`) reimplementava o laço em `main()` e nunca chamava `collect_call()`.

## 3. Situação no PX4 upstream (`upstream_px4/`)

Comparação entre o commit dos harnesses (8be2590c15, 04/06/2026) e o `main` do PX4 em 12/10/2026 (bf6603b42b).

| # | Módulo | No 8be2590c15 | No main atual | Consequência |
|---|---|---|---|---|
| 1 | Common.cpp | — | — | já retirado |
| 2 | externalChecks.cpp | sem limite em `num_events` | **corrigido**: ca1d36476a, PR #28598, 14/09/2026, "Reported by @REYu6" | issue seria duplicata |
| 3 | ft7_technologies.cpp | sem limite | **corrigido**: 3acf7935a0, PR #28600, 16/09/2026, "Reported by @kmm2003" | issue seria duplicata |
| 4 | sumd.cpp | `sumd_data[64]` com `length = 32` | **sem mudança** | off-by-one intra-struct, baixa severidade |
| 5 | logger.cpp | `write_info_multiple` igual | igual | sem issue (latente) |
| 6 | IridiumSBD.cpp | verificação com `_packet_length` | **corrigido**: 0c903d47d6, PR #28599, 16/09/2026, "Reported by @Finder16 and @Lqs66" | issue seria duplicata |
| 7 | bst.cpp | **já tinha as duas guardas**: e8e86a2e0f, 12/03/2026, antes do 8be2590c15 | igual | **falso positivo** em relação ao 8be2590c15 |
| 8 | MicroStrain.cpp | `_port[128]`, `max_len` até 128 | **sem mudança** (`_port[128]`) | real, baixa severidade (nome de porta ≥ 128 caracteres) |

Detalhe do candidato 7. O 8be2590c15 já tinha as duas guardas:

```
if (dev_info_reply.length >= sizeof(dev_info_reply)) { ... return -EIO; }
...
if (dev_info_reply.payload.dev_name_len >= sizeof(dev_info_reply.payload.dev_name)) {
        dev_info_reply.payload.dev_name_len = sizeof(dev_info_reply.payload.dev_name) - 1;
}
```

O harness `test_bst_dev_name.cpp` e o replay `replay_bst_probe_only.cpp` omitem as duas, e o comentário do harness diz "Não existe nenhuma checagem". Pelo critério da dissertação (guarda de produção omitida no harness), o candidato é **falso positivo**. As guardas vieram no commit e8e86a2e0f, de Ramon Roche, com a mensagem "fix(telemetry/bst): validate reply length and dev_name_len before use".

**Nenhuma issue ou PR foi aberta no upstream.** A ordem de submissão continua sendo decisão do Prof. Lucas.
