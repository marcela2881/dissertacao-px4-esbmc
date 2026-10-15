# Cronograma — Verificação dos módulos de saída do PX4

Início: 19/10/2026 (segunda-feira) · Duração prevista: 14 semanas · Término: 24/01/2027

Legenda de responsabilidade (conforme as Diretrizes de uso de LLMs do PPGEE):
- **[Você]**: decisão científica, execução e análise (hipótese, escolha, interpretação, escrita).
- **[Você + apoio]**: você conduz; o Claude pode ajudar na parte técnica (ler código, montar stubs e scripts, revisar).
- **[Professor]**: ponto de validação com o orientador.

---

## Fase 0 — Planejamento científico (semanas 1–2: 19/10 a 01/11)

| # | Tarefa | Quem | Entregável |
|---|---|---|---|
| 0.1 | Definir a hipótese e as questões de pesquisa (QPs) do novo trabalho | [Você] | 1 página com hipótese + 3 a 4 QPs |
| 0.2 | Escolher os ~10 módulos (sugestão: cadeia alocação → mixer → drivers) | [Você] | lista final com justificativa de cada módulo |
| 0.3 | Decidir o commit do PX4 (o mesmo `8be2590c15` ou um mais novo) | [Você] | commit fixado e registrado |
| 0.4 | Levantar e ler os datasheets e especificações (PCA9685/NXP, DShot, RoboClaw/Basicmicro, etc.) | [Você] | tabela: componente → propriedade → página do datasheet |
| 0.5 | Revisão bibliográfica: verificação de drivers, de cadeias de controle e de ponto flutuante | [Você] | fichamento dos artigos lidos |
| ✔ | **Reunião com o professor:** aprovar hipótese, QPs e módulos | [Professor] | — |

## Fase 1 — Infraestrutura (semanas 3–4: 02/11 a 15/11)

| # | Tarefa | Quem | Entregável |
|---|---|---|---|
| 1.1 | Clonar o PX4 no commit escolhido e compilar uma vez (`make px4_sitl`) para gerar os headers do uORB | [Você + apoio] | build funcionando |
| 1.2 | Criar a estrutura do repositório: `harnesses/`, `stubs/`, `scripts/`, `logs/` | [Você + apoio] | estrutura no GitHub |
| 1.3 | Escrever stubs comuns: I2C, serial, uORB, tempo/NuttX | [Você + apoio] | `stubs/` reutilizáveis |
| 1.4 | Script de execução que **registra a linha de comando, a versão do ESBMC e o solver em cada log** e calcula a mediana de 3 execuções | [Você + apoio] | `scripts/run_esbmc.sh` |
| 1.5 | **Prova de conceito no `pca9685_pwm_out`** compilando contra o código real | [Você + apoio] | 1 harness funcionando + relatório do que precisou de stub |
| ✔ | **Ponto de decisão:** a abordagem "código real" é viável? Se não, ajustar o método antes de seguir | [Você] + [Professor] | — |

## Fase 2 — Harnesses e verificação (semanas 5–10: 16/11 a 27/12)

Ritmo sugerido: ~2 módulos por semana. Para cada módulo, seguir sempre os mesmos passos:
1. ler o código do módulo e definir o que é stub;
2. escrever as propriedades (automáticas + datasheet/especificação, com a página citada);
3. rodar o ESBMC com `--unwinding-assertions` **e** sem (documentar a diferença);
4. registrar tempo, veredito e contraexemplos.

| Semana | Módulos (sugestão — ajustar conforme 0.2) | Quem |
|---|---|---|
| 5 (16–22/11) | `pca9685_pwm_out` (finalizar), `roboclaw` | [Você + apoio] |
| 6 (23–29/11) | `dshot`, `uavcan/actuators` | [Você + apoio] |
| 7 (30/11–06/12) | `voxl_esc`, `tap_esc` (ou `vertiq_io`) | [Você + apoio] |
| 8 (07–13/12) | `px4io` | [Você + apoio] |
| 9 (14–20/12) | `mixer_module` | [Você + apoio] |
| 10 (21–27/12) | `control_allocation` + `control_allocator` (1–2 tipos de aeronave) | [Você + apoio] |
| ✔ | **Reunião com o professor:** resultados parciais | [Professor] | 

## Fase 3 — Cadeia de saída e validação cruzada (semanas 11–12: 28/12 a 10/01)

| # | Tarefa | Quem | Entregável |
|---|---|---|---|
| 3.1 | Verificação composicional: o que a alocação garante vira hipótese do mixer; o que o mixer garante vira hipótese do driver | [Você + apoio] | harness da cadeia + resultado |
| 3.2 | Validação cruzada com o CBMC **usando a mesma pré-condição** do ESBMC | [Você + apoio] | logs comparáveis |
| 3.3 | Para cada contraexemplo: confrontar com o código de produção (reexecução ou leitura) | [Você] | classificação: real / falso positivo / só no modelo |
| 3.4 | Se houver correção: verificar o patch **com a pré-condição que reproduz a falha** | [Você + apoio] | log antes/depois |
| 3.5 | Relatar à comunidade PX4 apenas o que foi confirmado no código real | [Você] | Issues/PRs |
| 3.6 | Comparar com o artigo anterior: "cópia × código real" (ex.: reverificar o `pwm_out`) | [Você] | tabela comparativa |

## Fase 4 — Análise e escrita (semanas 13–14: 11/01 a 24/01)

| # | Tarefa | Quem | Entregável |
|---|---|---|---|
| 4.1 | Analisar os resultados e responder às QPs | [Você] | tabela de resultados + respostas |
| 4.2 | Escrever o artigo/capítulo **com suas palavras** | [Você] | rascunho completo |
| 4.3 | Revisão de gramática, coesão e conferência dos números com os logs | [Você + apoio] | versão revisada |
| 4.4 | Publicar o artefato (GitHub + Zenodo) com todos os harnesses, stubs, comandos e logs | [Você + apoio] | DOI do artefato |
| 4.5 | Declaração de uso de IA (o que foi usado e para quê) | [Você] | seção no texto |
| ✔ | **Reunião com o professor:** revisão final | [Professor] | — |

---

## Riscos e plano B

| Risco | Plano B |
|---|---|
| ESBMC não aceita o C++ do PX4 em algum módulo | stub da classe problemática e documentar o recorte |
| Ponto flutuante (alocação) sem veredito no tempo limite | testar Bitwuzla/Z3; reduzir o recorte; relatar como resultado |
| Atraso em algum módulo | reduzir de 10 para 8 módulos, mantendo a cadeia (alocação → mixer → 1 driver) |
| Nenhuma violação encontrada | resultado válido: "verificado sem violações até k sobre o código real" |

## Acompanhamento semanal

Toda sexta-feira, registrar em `plano/diario.md`:
- o que foi feito na semana;
- problemas encontrados;
- decisões tomadas e por quê;
- próximo passo.
