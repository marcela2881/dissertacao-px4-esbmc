# Nota para a dissertação: candidatos extras (guardar, ainda não incluir)

Registrado em 12/10/2026. O Prof. Lucas autorizou mencionar na dissertação os bugs que outras pessoas reportaram de forma independente, como evidência de que a técnica encontra defeitos reais. A autora pediu para **não incluir ainda**.

## Onde entraria

No Cap. 5 (`7-resultados.tex`) já existe uma seção desativada (`\iffalse`), "Aplicação a Módulos Adicionais do Contexto SAGRES" (`sec:sagres-adicionais`). O texto dela está **desatualizado**: trata o Common como achado e descreve um replay do logger que não vale mais. A ideia é substituir esse texto e ativar a seção.

A seção seguiria a decisão anterior do Prof. Lucas: os módulos extras entram como evidência de generalidade, **não** como ampliação da amostra de 12 módulos. As tabelas, o Resumo e o Abstract não mudam.

Quando a seção for incluída, a subseção "Construção dos harnesses com apoio de ferramenta de IA" também precisa de ajuste. Hoje ela diz que a conferência "capturou o único caso" de guarda omitida (o FIFO). Com o bst, passam a ser dois casos.

## Fatos (todos conferidos em 12/10/2026)

Commit testado: 8be2590c15 (04/06/2026). Os candidatos foram documentados no relatório ao orientador de **19/08/2026**, antes das correções upstream.

| # | Módulo | Resultado no 8be2590c15 | Situação no PX4 |
|---|---|---|---|
| 2 | externalChecks.cpp | violação (`num_events` até 255 indexa `events[5]`) | corrigido em 14/09/2026, commit ca1d36476a, PR #28598, relatado por outra pessoa |
| 3 | ft7_technologies.cpp | violação (contadores sem limite em `_readbuf[30]` e `_raw_*`) | corrigido em 16/09/2026, commit 3acf7935a0, PR #28600, relatado por outra pessoa |
| 6 | IridiumSBD.cpp | violação (a guarda compara com `_packet_length`, não com `buflen`) | corrigido em 16/09/2026, commit 0c903d47d6, PR #28599, relatado por outras pessoas |
| 4 | sumd.cpp | violação (`sumd_data[64]` com 32 canais, cai em `crc16_high`) | PR #29031 aberto em 08/10/2026 por outra pessoa, com a mesma análise (canal 32, `crc16_high`) |
| 8 | MicroStrain.cpp | violação (`_port[128]` com nome de porta ≥ 128 caracteres) | não reportado; impacto mínimo |
| 7 | bst.cpp | **falso positivo**: o harness omitia duas guardas existentes desde o commit e8e86a2e0f (março de 2026) | — |
| 1 | Common.cpp | retirado: a violação dependia de um índice que a guarda não permite | — |
| 5 | logger.cpp | nenhuma violação de memória para qualquer `key_len`; o truncamento é alcançável, mas gera só um registro ULog malformado | — |

- **Reverificação com as correções upstream (etapa vii):** os candidatos 2, 3 e 6 dão SUCCESSFUL no ESBMC e no CBMC.
- **Reprodução:** 49 de 49 comandos conferem em três ambientes:
  - ESBMC 8.1.0 e CBMC 5.95.1 (nuvem);
  - ESBMC 8.5.0 e CBMC 5.95.1 (fm07, Manchester, 12/10/2026);
  - ESBMC 8.4.0 e CBMC 6.8.0 (macOS, revisão do Prof. Lucas; versão anterior do pacote).
- **Sumd:** com `max_chan_count = 18` (RCInput e px4io, `RC_INPUT_MAX_CHANNELS` e `PX4IO_RC_INPUT_CHANNELS`), o canal 32 não é decodificado. Com 32 (RCTest), o canal 32 sai 530 em vez de 543.
- **Logs:** pasta `resultados_20261012_1456` na fm07 e `referencia_execucao_nuvem/` neste pacote.

Links:
- https://github.com/PX4/PX4-Autopilot/pull/28598
- https://github.com/PX4/PX4-Autopilot/pull/28599
- https://github.com/PX4/PX4-Autopilot/pull/28600
- https://github.com/PX4/PX4-Autopilot/pull/29031

## Mensagem central

Quatro dos oito candidatos foram confirmados de forma independente pela comunidade do PX4 em poucas semanas: três correções incorporadas e um PR aberto. A conferência contra o código de produção barrou o único falso positivo (bst) antes de ele virar uma issue pública.
