# Pendências da dissertação (atualizado em 23/11/2026)

## Guardado para a fase final (acabamento)
- **Portaria CNPq nº 2.664/2026**: acrescentar ao `referencias.bib` e citar na Declaração de Uso de IA.
  Dados conferidos: Portaria CNPq nº 2.664, de 6 de março de 2026, que institui a Política de Integridade
  na Atividade Científica do CNPq; publicada no DOU em 11/03/2026. Texto integral:
  https://abmes.org.br/arquivos/legislacoes/Portaria-cnpq-2664-2026-03-06.pdf
  Observação: o dispositivo exato sobre IA generativa ainda não foi conferido no texto integral.
- **Sigla SAGRES**: acrescentar à Lista de Siglas quando a autora informar o significado por extenso.

## Depende só do assistente
1. Método de inspeção (Resultados): ESBMC ou leitura; locais (statustext, RC Update).
2. Quatro pedidos só de texto: nº do datasheet; arquivos em `figuras/`; "nove dos onze logs"; justificativa de `--unwind` 10/15/300.
3. `Menezes2024` na Metodologia (frase do VTOL): trocar a citação (precisa da fonte da informação).
4. Pôster: taxonomia nova; decidir sobre a frase das 710 mil linhas.

## Respostas da autora
- Sumário com 2 ou 3 níveis (recomendado: 2).
- O que é a máquina "iartes".
- Confirmar a banca com o PPGEE.

## Logs que a autora roda no notebook (a partir de ~/repo_nuvem)
- `rodar_inconclusivos_wsl.sh` (Battery 16 células; GPS k=211; cada teste separado)
- `rodar_cbmc_dez.sh` (CBMC nos dez harnesses)
- `rodar_baseline_cppcheck.sh` e `gerar_evidencia.sh` (linha de base; QE2 em sumd/batmon/RoboClaw)
- anexar `roboclaw_uninit.tgz`

## Terceiros / depois
- Prof. Lucas: limite de páginas (corpo 104 ou total ~152); taxonomia; validação do GPS em outubro como uma das duas em hardware.
- Autora publica: nova versão no Zenodo (atualizar o DOI em 3 lugares); respostas nas issues após revisão do professor.
- Quando saírem: DOI do Wang2021; páginas/DOI do Rodrigues2026; ficha catalográfica.
- Hardware: FT7 e GPS na bancada (falta cabo USB de dados); RoboClaw na `main`.
- Limpeza final: remover os comentários `%% LC` do código-fonte (decisão do professor).
