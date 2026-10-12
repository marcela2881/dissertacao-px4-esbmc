# Re-verificação — agosto de 2026

Esta pasta reúne os logs gerados na re-execução feita em 19/08/2026, motivada
pelo parecer do orientador sobre a reprodução independente do artefato.

Os logs das rodadas originais permanecem em `3_resultados_ANTIGO/`, sem alteração. O
que está aqui é o que foi executado depois, com comandos corrigidos ou com
harnesses reescritos.

## Ambiente

Servidor `iartes` — Intel i7-13700K / 64 GB / Ubuntu 24.04
ESBMC v8.1.0, CBMC 5.95.1, g++ com ASan e UBSan

Mesmo ambiente das rodadas originais. A reprodução independente do orientador
(macOS 26.6 / aarch64 / ESBMC 8.4.0 / Bitwuzla 0.9.1 / CBMC 6.8.0 / Apple clang
21.0.0) não está reproduzida aqui — ela consta do relatório dele.

## O que foi re-executado e por quê

### Candidato 4 (sumd.cpp) — dois arquivos

`esbmc_sumd_unwind70.txt`
O comando original usava `--incremental-bmc`, sob o qual o k-induction diverge
no laço CRC16 aninhado e a execução estoura o timeout. Os laços têm cota
estática conhecida (65 chamadas de decode × 8 iterações de CRC), o que justifica
o unwind explícito. Resultado: `VERIFICATION FAILED`, `_rxlen = 64`, violação de
limite de array em `test_sumd_decode.cpp:162`.

`ubsan_sumd.txt`
A tabela do guia registrava "overflow" na coluna do AddressSanitizer, mas o
`asan_sumd.txt` da rodada original não contém relatório algum — o replay roda
até o fim e sai com código 0. A causa é que a escrita em `sumd_data[64]`
permanece dentro do mesmo objeto `malloc` (`ReceiverFcPacketHoTT`, 71 bytes;
`sumd_data` no offset 3, logo o índice 64 atinge o byte 67, campo `crc16_high`).
O ASan instrumenta as bordas da alocação, não os limites entre membros. O UBSan
detecta, porque o subscrito incide sobre array de tamanho estático.

### Candidato 5 (logger.cpp) — um arquivo

`esbmc_logger.txt` (e `esbmc_logger_resumo.txt`)
Não havia log algum para este candidato no pacote original — era a única linha
da tabela sem artefato de apoio. Com o comando do guia o ESBMC devolve
`VERIFICATION UNKNOWN`, porque o laço do harness tem 299 iterações e o k-step
default do `--incremental-bmc` é 50. Com `--max-k-step 320` o resultado é
`VERIFICATION SUCCESSFUL`, e a condição de avanço converge:
`Solution found by the forward condition; all states are reachable (k = 300)`.
Isso vale para o harness (laço literal de 299 iterações; `snprintf` modelado devolve valor irrestrito para `%s` simbólico): conclusão segura = sem violação de memória para os comprimentos modelados e qualquer `key_len` 0–255; não é prova sobre o `logger.cpp` real.

### Candidato 6 (IridiumSBD.cpp) — um arquivo

`cbmc_iridiumsbd.txt`
O guia registrava `UNKNOWN` para o CBMC, atribuindo ao laço interno do `memcpy`.
Na verdade o CBMC 5.95.1 sequer converte o arquivo: aborta com
`symbol 'ssize_t' is unknown / CONVERSION ERROR`. Com `-D'ssize_t=long'` e
`--unwind 400` ele decide e reporta `FAILED` em 2 de 10 propriedades, nas linhas
130 e 155 — as mesmas obtidas pelo orientador com o CBMC 6.8.0. Segue-se que o
fator determinante é o unwind, e não a versão da ferramenta.

### Candidato 7 (bst.cpp) — dois arquivos

`asan_bst_probe_only.txt` e `LEIA-ME_bst.txt`
O `replay_bst_real.cpp` não alcançava `bst_probe_real()`: o `main()` injetava o
contraexemplo numa cópia heap-alocada na linha 172 antes de chamar o `probe()`
na linha 179, e a escrita da linha 172 disparava o ASan, abortando o processo. O
log original `asan_bst_codigo_real.txt` termina em `replay_bst_real.cpp:172 in
main`, confirmando que o `probe()` verbatim nunca executou.

O `replay_bst_probe_only.cpp` preserva o arquivo original até o `main()` — as
145 primeiras linhas são idênticas byte a byte — e troca apenas o driver. O log
novo mostra o frame `bst_probe_real()` e o objeto `dev_info_reply` em `[48, 90)`.
Ver `LEIA-ME_bst.txt` para a comparação entre os dois.

### Candidato 8 (MicroStrain.cpp) — quatro arquivos

`esbmc_microstrain_v3.txt`, `esbmc_microstrain_v3_multi.txt`,
`cbmc_microstrain_v3.txt`, `asan_microstrain_v3.txt`

O harness v2 tinha dois problemas. O primeiro, apontado pelo orientador: a
asserção do `main()` era `assert(PORT_SIZE <= PORT_SIZE - 1)`, isto é
`128 <= 127`, uma contradição. Ela nunca chegou a ser alcançada — o ESBMC para
na primeira violação, que ocorria antes, na verificação automática de limites da
linha 38 — mas era ruído.

O segundo problema é o que motivou a reescrita: o v2 fixava
`const size_t max_len = PORT_SIZE;` por atribuição direta, sem `uart_port`, sem
`strnlen` e sem `min`. O verificador demonstrava apenas que `_port[128]` estoura
um array de 128 posições, o que é aritmética e não verificação. A pergunta
relevante — existe entrada que leve o driver a `max_len = 128`? — não era feita.

O v3 deixa `uart_port` não-determinístico byte a byte e calcula `strnlen` e `min`
como no driver. O contraexemplo passa a ser derivado da entrada:
`uart_port[0..127]` não-nulos, `uart_port[128] = '\0'`, logo `max_len = 128`.
O replay v3 confirma no ASan com `heap-buffer-overflow`, `WRITE of size 1`,
`0 bytes after 128-byte region`.

### Enumeração completa de propriedades — dois arquivos

`esbmc_bmi088_multi.txt`, `esbmc_bst_multi.txt`

Por padrão o ESBMC encerra na primeira violação. A flag `--multi-property` faz
com que verifique todas, mas não é compatível com `--incremental-bmc` — nessa
combinação a condição de avanço não converge e o resultado é UNKNOWN. Com
`--unwind` fixo funciona.

Resultados: BMI088 com 120 propriedades (118 verificadas, 2 violadas — a
temperatura e o FIFO); `bst` com 11 (9 verificadas, 2 violadas).

Isso tem consequência para a dissertação: a Tabela 5.1 registra `n/a` nas VCCs
do BMI088 e do GPS, com o rodapé atribuindo o fato a o BMC parar na primeira
violação. A explicação correta é outra — é a configuração `--incremental-bmc`
que impede a enumeração, não uma limitação do método.

## Arquivos de resumo

Dois logs passavam de 8 MB, quase inteiramente linhas de `Unwinding loop`. As
versões `*_resumo.txt` trazem o cabeçalho, um marcador explícito de corte com a
contagem de linhas omitidas, e o resultado final. Os originais permanecem.

## Adição v4 — Candidato 1 (Common.cpp), reexecução de 08/10/2026

Pasta `cand1_reexecucao_claude/`. **Gerada no ambiente da Claude (CBMC 5.95.1), não no `iartes`.** Repetir no `iartes` antes de citar.
Com `idx <= SIZE+4` (premissa do harness original): 5 de 10 propriedades falham. Com `idx <= SIZE` (invariante mantida pelo guard): 0 de 10 falham, tanto com SIZE 72 quanto 168. Achado retirado.
