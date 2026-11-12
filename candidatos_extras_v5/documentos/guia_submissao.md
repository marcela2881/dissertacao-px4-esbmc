# Como submeter ao PX4 (v5)

## O que deu errado antes

- **PR #27575 (BMI088):** misturou a correção do driver com mudanças de CI no mesmo PR e foi rejeitado.
- **Issues #26865 e #26866:** fechadas pelo bot como "not planned".
- **PR #27576:** está parado; o revisor pediu mudanças.

## Regras desta rodada

1. Nada é publicado sem a aprovação do Prof. Lucas.
2. Uma issue por defeito.
3. Primeiro a issue. O PR só vem depois do retorno dos mantenedores, no formato que eles indicarem.
4. Nada de CI/CD no mesmo PR da correção.
5. **Transparência sobre o método e sobre o uso de IA.** A regra antiga ("não mencionar que usou IA") foi removida. Ela contraria a Portaria CNPq nº 2.664/2026, e os próprios mantenedores do PX4 declaram `Assisted-by` nos commits. O texto proposto está em `declaracao_uso_IA.md`; a decisão é do Prof. Lucas.
6. Antes de abrir, procurar no GitHub do PX4 se já existe issue ou PR sobre o mesmo trecho, e rodar o `conferir_px4.sh` no dia para confirmar que o código ainda é o mesmo no `main`.
7. Colar só saída verbatim. As versões em `resultados_*/issues_prontas/` já vêm assim.
8. Uma issue por vez, com intervalo entre elas.

## O que reportar

| Candidato | Reportar? | Motivo |
|---|---|---|
| 4 sumd | sim | igual no `main`; baixa severidade |
| 8 MicroStrain | sim | igual no `main`; baixa severidade |
| 2, 3, 6 | não | já corrigidos no PX4 por outras pessoas (set/2026) |
| 7 bst | não | falso positivo |
| 1 Common | não | retirado |
| 5 logger | não | sem violação de memória |

Ordem sugerida, a confirmar com o Prof. Lucas: primeiro o sumd, que tem efeito observável no canal 32 e não depende de configuração; depois o MicroStrain.
