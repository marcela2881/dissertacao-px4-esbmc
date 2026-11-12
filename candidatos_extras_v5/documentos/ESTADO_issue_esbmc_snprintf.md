# Issue do ESBMC sobre o `snprintf`: estado em 2026-11-12

O rascunho `issue_esbmc_snprintf.md` **foi publicado** pela autora em
2026-10-12 como a issue 8524 do repositório `esbmc/esbmc`
("snprintf model returns a wrong length when the %s argument is built at run
time"). As menções "decisão do senhor" em `CORRECOES_v5.md` (itens A7 e a
lista final) estão superadas.

Registro transcrito da página da issue, colada pela autora em 2026-11-12:

- Reportado com ESBMC 8.5.0 (mesmo resultado em 8.1.0 e 8.4.0): alarme falso
  no programa reduzido `sondagem_snprintf.c`, com `snprintf` retornando 9 em
  vez de 10.
- Em 2026-10-13 o mantenedor (Lucas Cordeiro) confirmou que o problema também
  ocorria no `master`: `%s` de um vetor sem inicializador estático caía num
  limite pelo tamanho do objeto, e `snprintf` podia retornar qualquer valor em
  `[0, 10]`.
- Corrigido pelo PR 8563 ("[goto-symex] Pin sprintf's %s length to the
  argument's strlen"), commit `0ece769515d2e4f51fa8126a47d3ad813b0532ff`;
  issue fechada como concluída.
- O valor de retorno de `printf`/`fprintf` ficou fora dessa correção e passou
  a ser acompanhado na issue 8564 do mesmo repositório.

## Reteste do logger: não será feito

No comentário de 2026-10-13 o mantenedor sugeriu testar o ramo da correção no
código do logger do PX4 (candidato 5). Como o PR foi integrado e a issue
fechada sem esse teste, a autora decidiu em 2026-11-12 não fazê-lo. Na
dissertação o candidato 5 permanece como "nenhuma violação de memória", com a
ressalva de que o modelo de `snprintf` das versões usadas não exercitava o
truncamento.
