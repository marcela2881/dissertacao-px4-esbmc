# Pendências da dissertação — análise completa (23/11/2026)

Estado: 8 capítulos compilam sem erro; 151 páginas (corpo = 104; pré-textuais = 18 romanas; refs 106-115; apêndices ~116-132).
Caixas amarelas do professor abertas: 3 (todas em Resultados: GPS k=211, cada teste separado, CBMC nos dez). Fechadas em 23/11 com a evidência da autora: QE2, método de inspeção e linha de base (cppcheck). Comentários %% LC no código: 108 (a remover no fim).
Estimativas: M = tempo de máquina da autora (roda sozinho, pode deixar de noite); A = tempo ativo da autora; E = tempo do assistente; T = espera de terceiros. São estimativas, não medições.

## URGENTE (prazo externo)
| Item | Prazo | Estimativa |
|---|---|---|
| Pôster do SBESC 2026 (Salvador, 24 a 27/11): trecho do GPS ainda na taxonomia antiga; decidir a frase das 710 mil linhas | evento começa 24/11 | E 1-2 h; A 20 min para revisar |

## 1. Caixas abertas em Resultados (6)
| # | Caixa | O que falta | Estimativa |
|---|---|---|---|
| 1 | Linha de base (cppcheck) | FECHADA em 23/11 (Seção "Linha de Base com Análise Estática"; saída do terminal arquivada) | — |
| 2 | GPS com k = 211 | `inconclusivos.tgz`; GPS testes 0-3 não deram veredito em 900 s com Z3 (8.2.0 sem Boolector). Opções: (a) escrever como sem veredito; (b) rodar LIMITE=3600 (4 h); (c) compilar ESBMC 8.5 com Boolector (1-2 h de preparo) | M 4-5 h (opção b); E 1 h |
| 3 | Cada teste separado | `inconclusivos.tgz` cobre 7 testes. O que o professor quer é separar onde a 1ª falha mascara: IMU (6 testes) e GPS (5). Script novo `rodar_cada_teste.sh` para os 11 | E 1 h (script) + 1 h (texto); M 1-3 h |
| 4 | CBMC nos dez | rodar `rodar_cbmc_dez.sh` (20 execuções, 900 s cada no pior caso) | M 1-5 h; E 1 h + 30 min (Ameaças) |
| 5 | QE2 em sumd, batmon, RoboClaw | FECHADA em 23/11 (saída do terminal da autora arquivada) | — |
| 6 | Método de inspeção (statustext, RC Update) | FECHADA em 23/11 | — |

## 2. Textos que mudam quando os logs chegarem (revisão em cascata)
Resultados (configuração e Z3; seção CBMC; QE2; baseline; tabela de testes), Ameaças (defeitos no verificador: hoje diz CBMC só em 2 módulos; validade de conclusão), Metodologia (tabela de timeouts +900 s; tabela de versões +execuções de 23/11), Conclusão (QE2 parcial; limitações), Trabalhos Relacionados (frase do clang-tidy sem comparação), Introdução (se houver mudança de contribuição). Estimativa: E 2-3 h.

## 3. Qualidade e conformidade (assistente)
| Item | Estimativa |
|---|---|
| Tabela 3.1 (trabalhos relacionados) excede a página em 70 pt (aviso do LaTeX, pp. 23-25): conferir se corta; dividir em duas ou reduzir | E 30 min |
| 26 avisos de Underfull hbox (cosmético) | E 30 min |
| Auditoria final dos números (cada número do texto contra um log; tabelas de tempo, contagens, 22/12/10/7 módulos) | E 2 h (após os logs) |
| Matriz de atendimento dos 49 comentários do professor (comentário -> onde foi resolvido), para ele conferir | E 2 h |
| Revisão de inglês acadêmico do Abstract (sugestão do validador) | E 1 h |
| Wang2021: DOI (entrada tem título e anais, falta DOI) | E 15 min |
| Atualizar README/LOGS.md/REPRODUCAO.md do repositório com os logs novos e hashes; arquivar `roboclaw_uninit.tgz` | E 1,5 h |
| Decidir se apaga a pasta `_superado_nao_usar` e a quarentena de logs da nuvem | A 5 min; E 15 min |
| Limpeza final: remover 108 comentários %% LC e conferir compilação limpa | E 30 min (só após o professor fechar) |

## 4. Decisões do professor (bloqueiam o fechamento)
| # | Pergunta | Se a resposta for "sim/total" |
|---|---|---|
| 1 | Limite 100-120 páginas: corpo (104) ou total (151)? | Cortes obrigatórios: -30 a -40 páginas |
| 2 | Taxonomia de cinco categorias está de acordo? | Se pedir nome novo: troca global, E 1 h |
| 3 | A validação do GPS (out/2026) conta como uma das duas em hardware? | Se não: a segunda é o RoboClaw (já feita em 11/11); falta uma terceira |
| 4 | Autoriza cortes na Metodologia e no Exemplo do GPS? | E 3-4 h Metodologia + 2-3 h Resultados + 1 h Apêndice |
T (espera): depende da agenda dele; sugestão: enviar as 4 perguntas hoje em um e-mail só.

## 5. Plano de cortes (só se a decisão 1 for "total")
Metodologia 31 p. -> ~22 p. (Visão Geral 11 p. -> 5; Exemplo do GPS 5 p. -> 2; Seleção e Instrumentação sem repetição), Resultados 38 p. -> ~30 p. (Resultados Gerais e Discussão), Apêndice B (harness) 9 p. -> trechos (só se o professor aceitar, pois ele pediu "reproduzir aqui a versão corrigida"). Estimativa total: E 6-8 h em dois ciclos, mais conferência de referências cruzadas (E 1 h).

## 6. Seu lado, sem cabo (curto)
| Item | Estimativa |
|---|---|
| Confirmar a banca com o PPGEE | A 10 min |
| SAGRES por extenso (lista de siglas) | A 2 min |
| Publicar nova versão no Zenodo (só depois do último commit do harness) e me passar o DOI | A 30 min; E 15 min (3 lugares) |
| Postar nas issues após revisão do professor (RoboClaw: comentário opcional; já aberta e em triagem) | A 20 min |
| Prazos e requisitos de depósito do PPGEE (formato, ficha catalográfica, folha de aprovação assinada, verificação de similaridade) | A 1 h de consulta; T dias |
| Ficha catalográfica (biblioteca UFAM) | T dias a 2 semanas |
| Páginas e DOI do Rodrigues2026 (só após saírem os anais) | T semanas |

## 7. Bloqueado por falta de cabo USB de dados
| Item | Estimativa quando houver cabo |
|---|---|
| RoboClaw na main (refactor PR 29473), pedido na issue #29785 | compilar e gravar 1-2 h + ensaio 1 h + texto E 1 h |
| FT7 com ESP32 | A 3-4 h + E 1 h |
| GPS na bancada, sentido receptor -> placa | A 2 h + E 1 h |
Nenhum deles é necessário para o texto atual (estão como trabalho futuro).

## 8. Opcional
Reexecutar a primeira campanha no notebook (M 2-3 h; troca proveniência dos números do servidor para a sua máquina); comentário na issue do RoboClaw com o resultado do Valgrind (rascunho pronto, valor baixo porque o refactor da main já corrige).

## Resolvido (sai da lista)
Sumário em 2 níveis; número do datasheet conferido no PDF da Bosch (BST-BMI088-DS000-19 está correto); declaração de IA "mista"; status das issues (abertas, needs-triage); Menezes2024 substituído; Portaria CNPq incluída; 12 entradas não citadas removidas do .bib; nome da máquina removido dos documentos ativos; figuras removidas; Metodologia, Fundamentação, Trabalhos Relacionados, Introdução e Conclusão com 0 caixas.

## Caminho crítico estimado
Hoje: A roda cbmc_dez (M 1-5 h) e cppcheck/evidência (M < 30 min) em paralelo, e anexa os tgz. Amanhã: E integra tudo (E 5-7 h) e entrega o PDF para o professor. Depois: resposta do professor (T). Após as decisões: cortes (E 6-8 h, se for o caso), limpeza final (E 30 min), Zenodo/DOI (A 30 min), depósito (T). Total até "pronto para o professor": cerca de 1 dia de trabalho depois dos logs; até "pronto para depósito": mais 2-4 dias, dominados por resposta do professor e secretaria.
