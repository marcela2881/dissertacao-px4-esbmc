# Como submeter ao PX4 sem repetir os erros anteriores (v4)

## O que deu errado antes

- PR #27575 (BMI088): misturou fix de driver + CI no mesmo PR — rejeitado
- Issues #26865/#26866: fechadas pelo bot como "not planned"; remediação por clamping não é o padrão do projeto
- PR #27576: aberto, parado; mrpollo pediu mudanças

## Regras pra essa rodada

1. Uma Issue por bug — nunca dois bugs numa Issue só
2. Abrir Issue primeiro, esperar feedback, só aí abrir PR
3. Não incluir CI/CD no mesmo PR do fix
4. **DECISÃO DO PROF. LUCAS (pendente):** a regra anterior era "não mencionar que usou IA". Isso é omissão e pode contrariar a política do PX4 sobre contribuições assistidas por IA. Recomendação: em vez de omitir, descrever o **método** de forma factual ("verificação por BMC com ESBMC/CBMC + replay com sanitizadores; harnesses em <repo>") e, se a política do projeto exigir, declarar o uso de ferramentas de IA. Não publicar até o Lucas decidir.
5. Perguntar nos comentários da Issue qual é o padrão esperado antes de propor qualquer código

## Ordem recomendada (reconciliada com o rascunho das issues)

Conferir com o Lucas. Proposta: (1) bst.cpp — mais forte (ESBMC+CBMC+ASan no `probe()` verbatim); (2) IridiumSBD (faixa 331–340, causa clara); (3) MicroStrain (harness v3); depois ft7, externalChecks, sumd (baixa severidade). **Common.cpp: retirado. logger.cpp: sem issue.** Uma por vez.
