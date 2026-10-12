# Retorno sobre o parecer do pacote de candidatos (v5)

Marcela, 12/10/2026

Professor, muito obrigada pelo parecer e por ter reproduzido os oito candidatos. Segui as recomendações e organizei a resposta item a item no `CORRECOES_v5.md`. Aqui resumo o que mudou e o que depende da sua decisão.

## 1. Os logs agora saem de um único script

Como não tenho mais acesso ao iartes, troquei o `COPIAR_DO_IARTES.sh` por um `rodar_tudo.sh`. Ele gera todos os logs em qualquer máquina com ESBMC, CBMC e um compilador. Cada log traz no cabeçalho a máquina, a pasta, o comando exato e o código de saída. No fim, um `RESUMO.md` compara o esperado com o obtido em cada comando. Os resultados que envio junto foram gerados por esse script, na máquina e com as versões registradas no `ambiente.txt`.

## 2. Conferi os candidatos contra o código do PX4

O senhor apontou que a fidelidade dos harnesses ao PX4 não podia ser verificada sem a árvore do PX4 e um commit fixo. Fixei o commit testado (8be2590c15) e fiz um script, o `conferir_px4.sh`, que compara cada trecho copiado com esse commit e com o `main` do dia. Encontrei quatro coisas importantes:

| # | Módulo | O que encontrei |
|---|---|---|
| 2 | externalChecks | corrigido no PX4 em 14/09/2026 (PR #28598), a partir do relato de outra pessoa |
| 3 | ft7 | corrigido em 16/09/2026 (PR #28600), mesma situação |
| 6 | IridiumSBD | corrigido em 16/09/2026 (PR #28599), mesma situação |
| 7 | bst | **falso positivo**: o commit testado já tinha as duas guardas (de março de 2026), e o harness as omitia |

Nos três primeiros, as correções do PX4 confirmam que eram defeitos reais no commit testado. Reverifiquei os três harnesses com as correções aplicadas, e o resultado esperado é SUCCESSFUL (etapa vii). Para o bst, fiz um harness com as guardas, e ele também dá SUCCESSFUL.

Com isso, **restam dois candidatos para reportar: o sumd e o MicroStrain**. Os dois continuam iguais no `main` e são de baixa severidade:

- **sumd.** Rodei a função `sumd_decode()` real num quadro válido de 32 canais. A escrita fora do array cai no `crc16_high` e não sai do objeto. Com 32 canais decodificados (como no `RCTest`), o canal 32 sai com o byte do CRC no lugar do seu. No `RCInput`, que decodifica no máximo 18 canais, não achei efeito.
- **MicroStrain.** Exige um nome de porta com 128 caracteres ou mais no `-d`.

Os rascunhos das duas issues estão em `issues_prontas/`, já com a saída verbatim desta execução.

## 3. Logger (A6, A7, B15)

O senhor tinha razão: o k = 300 é só o laço concreto do harness. Reescrevi a nota para dizer apenas o que foi provado: nenhuma violação de memória para qualquer valor de `key_len`.

Para testar o truncamento em si, criei o modo `-DMODELO_KEY_LEN_NONDET`, com o retorno de `snprintf` livre e uma asserção explícita. O resultado é que o truncamento é alcançável e as propriedades de memória continuam valendo: o efeito seria um registro ULog malformado, não uma escrita fora dos limites.

A sondagem do `snprintf` (seção 5.4 do parecer) se reproduziu também no ESBMC 8.1 e no CBMC 5.95.1.

## 4. O que depende do senhor

1. **Declaração de uso de IA (A11).** Removi a regra de não mencionar o uso de IA. Propus um parágrafo nas issues e um texto para a dissertação (`declaracao_uso_IA.md`), seguindo a Portaria CNPq nº 2.664/2026. Os próprios mantenedores do PX4 registram `Assisted-by` nos commits. Peço sua orientação sobre o texto.
2. **Se e quando abrir as issues do sumd e do MicroStrain**, e em que ordem.
3. **Issue no ESBMC sobre o modelo de `snprintf`.** O rascunho está em `issue_esbmc_snprintf.md`.
4. **A "segunda ressalva" (C9)** do seu retorno anterior, que não chegou até mim.

Fico à disposição para qualquer ajuste.
