# Correções v4 (08/10/2026)

O v3 não foi alterado. Cada item abaixo corresponde a um achado da revisão crítica (review_report.md).

## O que foi corrigido neste pacote

| Arquivo | Mudança |
|---|---|
| `LEIA-ME_v4.md` | Candidato 1 retirado; logger com redação cautelosa; contagem: 6 achados, 1 retirado, 1 sem violação |
| `guia_reproducao.md` | Caminhos ANTIGO/ATUAL; tabela linha 8 = MicroStrain v3 FAILED/FAILED/heap-overflow; notas duplicadas (sumd, logger) removidas; `-D'ssize_t=long'` só no CBMC 5.95.1 e linha CBMC contraditória do IridiumSBD removida; ressalva sobre `--unwinding-assertions`; seção única de Ambiente; replay do MicroStrain aponta para o v3; IridiumSBD descrito corretamente (guarda compara com `_packet_length`, faixa 331–340) |
| `6_documentos/issues_px4_rascunho.md` | Issue 2 (Common) retirada; Issue 3: SIZE_MAX é marcador interno do ESBMC; Issue 4: removida a afirmação "redzone de 42 bytes", leitura 34 (Linux) / 37 (macOS), stack e não heap; Issue 5: estado derivado da entrada; Issue 6: causa e faixa 331–340; Issue 7: linha 184 vs 162 sinalizada; commit `8be2590c15`; placeholder `<DOI-NOVO>` |
| `5_submissao_px4/guia_submissao.md` | Regra 4 ("não mencionar IA") marcada como decisão do Lucas, com recomendação de declarar o método; ordem reconciliada |
| `5_submissao_px4/issue_bst.md` | Marcado OBSOLETO |
| `4_resultados_ATUAL/LEIA-ME.md`, `LEIA-ME_bst.txt` | logger sem generalização; frase da redzone corrigida; proveniência do `asan_bst_probe_only.txt` (ambiente Claude, não iartes) |
| `4_resultados_ATUAL/COPIAR_DO_IARTES.sh` | nomes/caminhos corretos, sem `2>/dev/null`, `set -euo pipefail`, sai com erro se faltar arquivo |
| `4_resultados_ATUAL/cand1_reexecucao_claude/` | 4 harnesses + logs do CBMC 5.95.1 do candidato 1 |
| `6_documentos/relatorio_para_LC.md` | logger não é "conclusivo" |

## Candidato 1 — resultado da reexecução (CBMC 5.95.1, ambiente Claude)

| Premissa do índice | SIZE 72 | SIZE 168 |
|---|---|---|
| `0 <= idx <= SIZE+4` (harness original) | 5 de 10 falham | 5 de 10 falham |
| `0 <= idx <= SIZE` (invariante do guard) | 0 de 10 | 0 de 10 |

## O que AINDA precisa ser rodado por você (no iartes/notebook)

1. Repetir o candidato 1 no iartes (os logs aqui são do ambiente Claude).
2. UBSan do sumd: log completo com código de saída.
3. bst: ASan no iartes (o log atual veio do ambiente Claude) e `-fsanitize=local-bounds`.
4. externalChecks: `--generate-testcase`.
5. ft7: replay chamando `collect_call()`.
6. GPS: patch com comparação CBMC justa (para a dissertação).
7. Subir medianas_logs, replays e logs de sanitizadores; criar nova tag Zenodo e trocar `<DOI-NOVO>`.
8. **Revogar os tokens do GitHub que estavam no zip do repositório** e removê-los do histórico.
9. Decisões do Lucas: regra sobre IA e ordem das issues.
10. Colar nas issues a saída verbatim dos comandos (marcado `<paste verbatim output here>`).
