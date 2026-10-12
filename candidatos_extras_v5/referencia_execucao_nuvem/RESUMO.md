# Resumo da execucao

- data: 2026-10-12T12:50:01Z
- maquina: vm
- ESBMC 8.1.0, CBMC 5.95.1, g++ (Ubuntu 13.3.0-6ubuntu2~24.04.1) 13.3.0
- PX4 main conferido: 3fc3b141d7 (2026-10-12)

| Candidato | Log | Esperado | Obtido | Confere | O que e |
|---|---|---|---|---|---|
| 1_Common_RETIRADO | esbmc_orig_72.txt | FAILED | FAILED | sim | premissa antiga idx <= SIZE+4 (SIZE 72; 168 e o tamanho real) |
| 1_Common_RETIRADO | cbmc_orig_72.txt | FAILED | FAILED | sim | premissa antiga idx <= SIZE+4 (SIZE 72) |
| 1_Common_RETIRADO | esbmc_orig_168.txt | FAILED | FAILED | sim | premissa antiga idx <= SIZE+4 (SIZE 168; 168 e o tamanho real) |
| 1_Common_RETIRADO | cbmc_orig_168.txt | FAILED | FAILED | sim | premissa antiga idx <= SIZE+4 (SIZE 168) |
| 1_Common_RETIRADO | esbmc_inv_72.txt | SUCCESSFUL | SUCCESSFUL | sim | invariante da guarda idx <= SIZE (SIZE 72; 168 e o tamanho real) |
| 1_Common_RETIRADO | cbmc_inv_72.txt | SUCCESSFUL | SUCCESSFUL | sim | invariante da guarda idx <= SIZE (SIZE 72) |
| 1_Common_RETIRADO | esbmc_inv_168.txt | SUCCESSFUL | SUCCESSFUL | sim | invariante da guarda idx <= SIZE (SIZE 168; 168 e o tamanho real) |
| 1_Common_RETIRADO | cbmc_inv_168.txt | SUCCESSFUL | SUCCESSFUL | sim | invariante da guarda idx <= SIZE (SIZE 168) |
| 2_externalChecks | esbmc.txt | FAILED | FAILED | sim | comando do guia |
| 2_externalChecks | esbmc_multi.txt | FAILED | FAILED | sim | contagem de propriedades, bound fixo (B16) |
| 2_externalChecks | esbmc_correcao.txt | SUCCESSFUL | SUCCESSFUL | sim | harness com a correcao upstream ca1d36476a |
| 2_externalChecks | cbmc.txt | FAILED | FAILED | sim | com trace para concretizar num_events (B6) |
| 2_externalChecks | cbmc_correcao.txt | SUCCESSFUL | SUCCESSFUL | sim | harness com a correcao upstream |
| 2_externalChecks | asan.txt | ASAN | ASAN | sim | replay com num_events=255 tirado do trace do CBMC |
| 3_ft7 | esbmc.txt | FAILED | FAILED | sim | comando do guia (a 1a propriedade impressa varia com a versao, B20) |
| 3_ft7 | esbmc_multi.txt | FAILED | FAILED | sim | contagem de propriedades, bound fixo (B16) |
| 3_ft7 | esbmc_correcao.txt | SUCCESSFUL | SUCCESSFUL | sim | harness com a correcao upstream 3acf7935a0 |
| 3_ft7 | cbmc.txt | FAILED | FAILED | sim |  |
| 3_ft7 | cbmc_correcao.txt | SUCCESSFUL | SUCCESSFUL | sim | harness com a correcao upstream |
| 3_ft7 | ubsan_cenario1.txt | UBSAN | UBSAN | sim | replay chama collect(): 2 leituras de 19 bytes sem \n |
| 3_ft7 | ubsan_cenario2.txt | UBSAN | UBSAN | sim | replay chama collect(): campo speed sem delimitador |
| 3_ft7 | ubsan_cenario1_correcao.txt | NADA | NADA | sim | mesmo replay com as guardas do upstream |
| 3_ft7 | ubsan_cenario2_correcao.txt | NADA | NADA | sim | mesmo replay com as guardas do upstream |
| 3_ft7 | asan_cenario1.txt | NADA | NADA | sim | ASan nao ve (escrita dentro do objeto); ver _interval corrompido na saida |
| 4_sumd | esbmc.txt | FAILED | FAILED | sim | comando do guia |
| 4_sumd | cbmc.txt | FAILED | FAILED | sim |  |
| 4_sumd | ubsan_32.txt | UBSAN | UBSAN | sim | sumd_decode() real, quadro de 32 canais |
| 4_sumd | efeito_32.txt | NADA | NADA | sim | sem sanitizador: efeito no canal 32 (max_chan_count=32, como no RCTest) |
| 4_sumd | efeito_18.txt | NADA | NADA | sim | sem sanitizador: max_chan_count=18 (como no RCInput) |
| 4_sumd | asan_32.txt | NADA | NADA | sim | ASan nao ve (escrita dentro do struct) |
| 5_logger | esbmc.txt | SUCCESSFUL | SUCCESSFUL | sim | comando do guia; k=300 e so o laco concreto (A6) |
| 5_logger | esbmc_key_len.txt | FAILED | FAILED | sim | key_len livre: so a assercao do truncamento deve falhar (A7) |
| 5_logger | esbmc_sondagem.txt | FAILED | FAILED | sim | sondagem do modelo de snprintf (A7, secao 5.4) |
| 5_logger | cbmc_sondagem.txt | OUTRO | FAILED | sim | mesma sondagem no CBMC (so registro) |
| 5_logger | sondagem_libc.txt | NADA | NADA | sim | mesma sondagem com a libc real (deve imprimir 10) |
| 6_IridiumSBD | esbmc.txt | FAILED | FAILED | sim | comando do guia (sem -D ssize_t: harness inclui sys/types.h) |
| 6_IridiumSBD | esbmc_multi.txt | FAILED | FAILED | sim | contagem de propriedades, bound fixo (B16) |
| 6_IridiumSBD | esbmc_correcao.txt | SUCCESSFUL | SUCCESSFUL | sim | harness com a correcao upstream 0c903d47d6 |
| 6_IridiumSBD | cbmc.txt | FAILED | FAILED | sim |  |
| 6_IridiumSBD | cbmc_correcao.txt | SUCCESSFUL | SUCCESSFUL | sim | harness com a correcao upstream |
| 6_IridiumSBD | asan.txt | ASAN | ASAN | sim | replay com payload_len=138 e second_buflen=332 tirados do log do ESBMC (B4) |
| 7_bst_FALSO_POSITIVO | esbmc_sem_guardas.txt | FAILED | FAILED | sim | harness ANTIGO, sem as guardas de producao (so rastreio) |
| 7_bst_FALSO_POSITIVO | esbmc_fiel.txt | SUCCESSFUL | SUCCESSFUL | sim | harness com as guardas do 8be2590c15 |
| 7_bst_FALSO_POSITIVO | cbmc_fiel.txt | SUCCESSFUL | SUCCESSFUL | sim | harness com as guardas do 8be2590c15 |
| 8_MicroStrain | esbmc.txt | FAILED | FAILED | sim | comando do guia |
| 8_MicroStrain | esbmc_multi.txt | FAILED | FAILED | sim | contagem de propriedades, bound fixo (B16) |
| 8_MicroStrain | cbmc.txt | FAILED | FAILED | sim |  |
| 8_MicroStrain | asan.txt | ASAN | ASAN | sim | replay: 128 bytes nao nulos |

Contagens de propriedades (ultima linha de contagem de cada log;
ESBMC 8.1 imprime 'Properties: ...', ESBMC 8.4 e CBMC imprimem '** N of M ...'):

- 1_Common_RETIRADO/cbmc_inv_168.txt: `** 0 of 16 failed (1 iterations)`
- 1_Common_RETIRADO/cbmc_inv_72.txt: `** 0 of 16 failed (1 iterations)`
- 1_Common_RETIRADO/cbmc_orig_168.txt: `** 5 of 16 failed (2 iterations)`
- 1_Common_RETIRADO/cbmc_orig_72.txt: `** 5 of 16 failed (2 iterations)`
- 1_Common_RETIRADO/esbmc_inv_168.txt: `Properties: 426 verified ✓ 426 passed`
- 1_Common_RETIRADO/esbmc_inv_72.txt: `Properties: 426 verified ✓ 426 passed`
- 1_Common_RETIRADO/esbmc_orig_168.txt: `Properties: 427 verified ✓ 325 passed, ✓ 98 skipped, ✗ 4 failed`
- 1_Common_RETIRADO/esbmc_orig_72.txt: `Properties: 427 verified ✓ 325 passed, ✓ 98 skipped, ✗ 4 failed`
- 2_externalChecks/cbmc.txt: `** 3 of 33 failed (2 iterations)`
- 2_externalChecks/cbmc_correcao.txt: `** 0 of 39 failed (1 iterations)`
- 2_externalChecks/esbmc_multi.txt: `Properties: 217 verified ✓ 203 passed, ✓ 7 skipped, ✗ 7 failed`
- 3_ft7/cbmc.txt: `** 6 of 80 failed (6 iterations)`
- 3_ft7/cbmc_correcao.txt: `** 0 of 86 failed (1 iterations)`
- 3_ft7/esbmc_multi.txt: `Properties: 2053 verified ✓ 1837 passed, ✓ 210 skipped, ✗ 6 failed`
- 4_sumd/cbmc.txt: `** 2 of 38 failed (2 iterations)`
- 5_logger/cbmc_sondagem.txt: `** 1 of 2 failed (2 iterations)`
- 5_logger/esbmc_key_len.txt: `Properties: 105 verified ✓ 104 passed, ✗ 1 failed`
- 6_IridiumSBD/cbmc.txt: `** 3 of 58 failed (2 iterations)`
- 6_IridiumSBD/cbmc_correcao.txt: `** 0 of 57 failed (1 iterations)`
- 6_IridiumSBD/esbmc_multi.txt: `Properties: 29 verified ✓ 28 passed, ✗ 1 failed`
- 7_bst_FALSO_POSITIVO/cbmc_fiel.txt: `** 0 of 52 failed (1 iterations)`
- 8_MicroStrain/cbmc.txt: `** 2 of 30 failed (2 iterations)`
- 8_MicroStrain/esbmc_multi.txt: `Properties: 2251 verified ✓ 2249 passed, ✗ 2 failed`
