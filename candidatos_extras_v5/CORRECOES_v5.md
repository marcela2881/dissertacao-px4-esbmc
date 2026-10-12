# Correções v5 (12/10/2026): resposta ao parecer, item a item

O parecer (review_report) foi escrito sobre o pacote v3. O v4 já tinha resolvido parte dos itens. O v5 resolve o restante e reorganiza o pacote.

Desde o parecer, três fatos novos mudam o alcance de vários itens:

1. **Três candidatos já foram corrigidos no PX4 por outras pessoas.** São o 2 (externalChecks, ca1d36476a), o 3 (ft7, 3acf7935a0) e o 6 (IridiumSBD, 0c903d47d6), todos em setembro de 2026. Nenhuma issue deve ser aberta para eles. Os itens do parecer sobre o texto das issues 2, 3 e 6 deixam de ter objeto.
2. **O candidato 7 (bst) é falso positivo.** O commit testado (8be2590c15) já tinha as duas guardas que o harness omitia; elas vieram do commit e8e86a2e0f, de março de 2026. A issue do bst não deve ser aberta.
3. **Os logs não dependem mais do iartes.** O `rodar_tudo.sh` gera todos eles em qualquer máquina com ESBMC, CBMC e um compilador C++. Cada log registra a máquina, o comando e as versões.

Restam dois candidatos para reportar ao PX4: o **4 (sumd)** e o **8 (MicroStrain)**. Os dois continuam iguais no `main`.

## Prioridade alta

| Item | O que o parecer pediu | O que foi feito no v5 |
|---|---|---|
| A1 | log do candidato 5 | `rodar_tudo.sh` gera `5_logger/esbmc.txt` |
| A2 | logs do MicroStrain v3 | gera `8_MicroStrain/esbmc.txt`, `esbmc_multi.txt`, `cbmc.txt` e `asan.txt` |
| A3 | log do CBMC do IridiumSBD | gera `6_IridiumSBD/cbmc.txt` |
| A4 | logs do sumd (ESBMC `--unwind 70` e UBSan) | gera `4_sumd/esbmc.txt` e `ubsan_32.txt` |
| A5 | `COPIAR_DO_IARTES.sh` | substituído pelo `rodar_tudo.sh`, que gera os logs em vez de copiá-los, e sai com código ≠ 0 se algo divergir; o antigo foi para `_superado_nao_usar/` |
| A6 | logger: retirar "não é limitada ao bound" | cabeçalho do harness e LEIA-ME reescritos: o k = 300 é o laço concreto; o resultado vale para "nenhuma violação de memória para qualquer `key_len`" |
| A7 | logger: o modelo de `snprintf` não exercita o truncamento | ver detalhe abaixo da tabela |
| A8 | tabela "O que esperar" (candidato 8) | o `guia_reproducao.md` foi substituído pelo `RESUMO.md` gerado, com esperado × obtido de cada comando |
| A9 | comandos do guia contra a árvore real | os comandos estão no `rodar_tudo.sh` e rodam sobre `candidatos/N_nome/` |
| A10 | cadeia causal do candidato 6 | corrigida no cabeçalho do harness: a causa é a guarda comparar com `_packet_length` e não com `buflen`; o underflow vem do mesmo pressuposto e ocorre depois. A issue não será aberta (corrigido upstream) |
| A11 | regra "não mencionar IA" | regra removida do `guia_submissao.md`; as issues trazem um parágrafo "AI assistance" e há texto de declaração para a dissertação (`documentos/declaracao_uso_IA.md`). **A decisão final é do senhor** |
| A12 | `issue_bst.md` | removido; o bst é falso positivo (ver `candidatos/7_bst_FALSO_POSITIVO/LEIA-ME.txt`) |

Detalhe do A7:

- `sondagem_snprintf.c` reproduz o problema. O script roda a sondagem no ESBMC, no CBMC e na libc real.
- O harness ganhou o modo `-DMODELO_KEY_LEN_NONDET`: `key_len` livre e uma asserção explícita do truncamento. Resultado esperado: o truncamento é alcançável (a asserção falha) e as propriedades de memória passam.
- O rascunho da issue para o ESBMC está em `documentos/issue_esbmc_snprintf.md`. Abrir ou não é decisão do senhor.

## Prioridade média

| Item | O que foi feito no v5 |
|---|---|
| B1 | cada log registra máquina, pasta e comando no cabeçalho |
| B2 | as issues usam só a saída verbatim, inserida pelo script a partir dos logs |
| B3 | os logs são gerados a partir dos arquivos entregues, então os números de linha batem |
| B4 | o replay do IridiumSBD recebe `payload_len` e `second_buflen` tirados do log do ESBMC da mesma execução |
| B5 | candidato 1 retirado; sem replay |
| B6 | `num_events` sai do trace do CBMC e é passado ao replay. No ESBMC 8.1 o `--generate-testcase` não concretiza esse campo; isso foi verificado em 12/10 e registrado |
| B7 | a issue 3 não será aberta (corrigido upstream) |
| B8 | replay do ft7 reescrito: chama o laço de `collect()` copiado do PX4, com os membros na ordem do `.hpp` |
| B9 | a tabela do LEIA-ME tem a coluna "estado inicial". Os harnesses 6 e 8 derivam o estado da entrada; no 6, o `assume(second_buflen > _packet_length)` saiu |
| B10, B11 | o guia antigo foi para `_superado_nao_usar/` |
| B12 | o harness do IridiumSBD inclui `<sys/types.h>`; nenhum `-D'ssize_t=long'` em lugar nenhum |
| B13 | todos os comandos do CBMC têm `--unwinding-assertions`, com bound que cobre os laços (externalChecks: 260; IridiumSBD: 345) |
| B14, C1 | nomes canônicos (`harness.cpp`, `replay.cpp`, logs `esbmc.txt`/`cbmc.txt`/…) e `_superado_nao_usar/` |
| B15 | logger com asserção explícita sobre `msg_size` |
| B16 | os candidatos com violação têm `esbmc_multi.txt` com `--unwind` fixo; a exceção é o sumd, em que o `--multi-property` com `--unwind 70` passa de 15 minutos, e a contagem vem do CBMC. ESBMC 8.1 imprime `Properties: N verified …`; 8.4 imprime `** N of M properties failed` |
| B17 | é diferença de versão. No mesmo harness v3, o CBMC 5.95.1 deu `2 of 8 failed` (nuvem, 12/10/2026) e o 6.8.0 do senhor deu `2 of 18`; as duas falhas são as mesmas. O v5 acrescenta `--pointer-check`, então o denominador muda de novo; vale o que estiver no log da execução |
| B18 | externalChecks: a asserção agora é sobre o índice acessado (`i < 5`), não sobre a entrada |
| B19 | uma única seção "Ambiente" no LEIA-ME, com o que muda entre versões |
| B20 | registrado no cabeçalho do harness do ft7 |
| B21 | `ORB_QUEUE_LENGTH = 16` no PX4 (msg/versioned/Event.msg), portanto o buffer real tem 168 bytes. O harness com 168 foi rodado junto com o de 72 e dá o mesmo resultado |
| B22 | numeração corrigida (sumd = 4, logger = 5) |

## Prioridade baixa

| Item | O que foi feito no v5 |
|---|---|
| C2, C3 | documentos antigos em `_superado_nao_usar/` |
| C4 | replay do IridiumSBD com cabeçalho |
| C5 | MicroStrain: comentário sobre `uart_port[128]` corrigido; `memcpy` trocado por `strncpy`, como no driver |
| C6 | o log de 8,5 MB do sumd (timeout) foi truncado para as primeiras e últimas 200 linhas |
| C7 | sem `free()`/`delete` depois de corrupção nos replays |
| C8 | `ULOG_MSG_HEADER_LEN` removido |
| C9 | "segunda ressalva": continua pendente, depende do senhor |
| C10 | `LICENSE` (MIT) incluído; commit testado = 8be2590c15; o `conferir_px4.sh` registra o hash do `main` do dia; o DOI do Zenodo sai quando subirmos o v5 |
| C11 | replay do externalChecks com a mesma struct do harness |
| C12 | removida a afirmação sobre `local-bounds` (era da issue do bst, que não será aberta) |

## Fora do pacote

- **Issue no ESBMC sobre o `snprintf`:** rascunho em `documentos/issue_esbmc_snprintf.md`; decisão do senhor.
- **Contagens para a Tabela 5.1:** a regra do B16 vale para elas também.
