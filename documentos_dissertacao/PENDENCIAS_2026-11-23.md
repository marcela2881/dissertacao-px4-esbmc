# Pendências da dissertação (atualizado em 23/11/2026, fim do dia)

Estado: 8 capítulos compilam sem erro (151 páginas com as caixas visíveis; corpo = 104 páginas).
Caixas amarelas do professor abertas: 6, todas em Resultados.

## 1. Caixas abertas em Resultados
| Caixa | Falta | Quem |
|---|---|---|
| Linha de base (analisador estático) | log de `rodar_baseline_cppcheck.sh` | autora roda; assistente escreve |
| GPS com k = 211 | `inconclusivos.tgz` (GPS testes 0-3 ficaram sem veredito em 900 s) | autora anexa; assistente escreve |
| Cada teste do main separadamente | `inconclusivos.tgz` (VTOL 0 e 1 verificados; EKF2 teste 1 sem veredito) | idem |
| CBMC nos dez harnesses | `cbmc_dez.tgz` (script consertado: 8 como C, Expo e MAVLink como C++11) | autora roda de ~/repo_nuvem |
| QE2: testes unitários em sumd, batmon, RoboClaw | conteúdo de `evidencia_teste_unitario_e_inspecao.txt` | autora cola |
| Método de inspeção (statustext, RC Update) | mesmo arquivo de evidência | assistente escreve após receber |

## 2. Logs a enviar
- `inconclusivos.tgz` (já gerado: `tar czf inconclusivos.tgz verificacao_pendencias_2026-11/logs_inconclusivos`)
- `cbmc_dez.tgz` (rodar de novo: `git pull` em ~/repo_nuvem e `nohup bash cbmc_dez_harnesses_2026-11/rodar_cbmc_dez.sh`)
- log do cppcheck baseline; saída do `gerar_evidencia.sh`
- `roboclaw_uninit.tgz`

## 3. Respostas da autora
- Significado de SAGRES por extenso (lista de siglas)
- Número correto do datasheet BMI088 (ler na capa do PDF da Bosch; provável BST-BMI088-DS001-<rev>, hoje o texto usa DS000-19)
- (decidido: sumário com 2 níveis)
- Confirmar a banca com o PPGEE

## 4. Assistente (sem depender de terceiros)
- Trocar as citações de Menezes2024 que restam na Metodologia (nondet, --no-library/VTOL, propriedades automáticas) por Cordeiro2012 / documentação do ESBMC
- Corrigir número do datasheet (.bib, Metodologia, Resultados) quando a autora informar
- Portaria CNPq nº 2.664/2026 no .bib e na Declaração de IA; sigla SAGRES na lista
- Remover 11 entradas não citadas do .bib; limpeza final dos comentários `%% LC` (quando o professor fechar)
- Reescrever Resultados/Conclusão com os logs novos (QE2, baseline, CBMC, testes separados)

## 5. Professor
- Limite de 100 a 120 páginas: corpo (104) ou total (151)?
- Taxonomia aplicada está de acordo?
- A validação do GPS em outubro conta como uma das duas em hardware?
- Se o total conta: cortes (Visão Geral e Exemplo do GPS na Metodologia; Metodologia e Resultados têm repetições)

## 6. Terceiros / tempo
- Autora publica nova versão no Zenodo (depois do último commit do harness) e atualiza o DOI em 3 lugares
- Autora posta respostas nas issues após revisão do professor (RoboClaw: comentário opcional)
- DOI do Wang2021; páginas e DOI do Rodrigues2026; ficha catalográfica
- figuras/: renomear imagematual.jpeg; remover bug1_temperatura.jpeg, bug_gps.png, bug2_fifo.jpeg (conferir no Overleaf)
- Hardware: FT7 e GPS na bancada (falta cabo USB de dados); RoboClaw na main (PR 29473)

## 7. Opcional
- Reexecutar a primeira campanha no notebook; atualizar o pôster (taxonomia; frase das 710 mil linhas)
