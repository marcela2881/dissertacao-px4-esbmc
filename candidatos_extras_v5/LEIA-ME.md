# Pacote v5: candidatos a defeito no PX4 (12/10/2026)

Este pacote traz os oito candidatos a defeito no PX4 Autopilot verificados com ESBMC, CBMC e sanitizadores. Ele foi corrigido segundo o parecer do Prof. Lucas; o item a item está em `CORRECOES_v5.md`.

**Um comando gera todas as evidências:**

```
./rodar_tudo.sh
```

O script cria a pasta `resultados_<data>/` com:

- um log por comando, cada um com cabeçalho (data, máquina, pasta, comando exato) e código de saída;
- o `RESUMO.md`, com o esperado e o obtido de cada comando;
- as duas issues já preenchidas com a saída verbatim da sua execução.

## Situação dos oito candidatos

Código testado: PX4 commit `8be2590c15` (04/06/2026). O `conferir_px4.sh`, chamado pelo script, compara cada trecho com o `main` do dia.

| # | Módulo | Estado inicial no harness | Resultado no 8be2590c15 | No PX4 hoje | Issue? |
|---|---|---|---|---|---|
| 1 | Common.cpp | assumido (`idx ≤ SIZE+4`) | some com a invariante da guarda | — | **não** (retirado) |
| 2 | externalChecks.cpp | derivado (`num_events` livre) | violação | corrigido em ca1d36476a (PR #28598) | **não** (duplicata) |
| 3 | ft7_technologies.cpp | derivado (bytes livres) | violação | corrigido em 3acf7935a0 (PR #28600) | **não** (duplicata) |
| 4 | **sumd.cpp** | derivado (quadro legítimo de 32 canais) | violação, dentro do objeto | **igual** | **sim** (baixa severidade) |
| 5 | logger.cpp | `key_len` livre | nenhuma violação de memória | igual | não |
| 6 | IridiumSBD.cpp | derivado (header e tamanho livres; v5) | violação | corrigido em 0c903d47d6 (PR #28599) | **não** (duplicata) |
| 7 | bst.cpp | — | **falso positivo**: o harness omitia duas guardas de produção | igual | **não** |
| 8 | **MicroStrain.cpp** | derivado (`uart_port` livre) | violação | **igual** | **sim** (baixa severidade) |

Os candidatos 2, 3 e 6 continuam válidos como resultado da pesquisa: no commit testado eram violações reais. Elas foram confirmadas depois pelas correções do próprio PX4. O script reverifica cada um com a correção upstream aplicada (`-DCORRECAO_UPSTREAM`), e o resultado esperado é SUCCESSFUL.

## Ambiente

Você precisa de:

- **ESBMC**: binário de release em https://github.com/esbmc/esbmc/releases (Linux e macOS);
- **CBMC**: `brew install cbmc` no macOS, ou o `.deb` em https://github.com/diffblue/cbmc/releases;
- **um compilador C++**: `clang++` (Xcode no macOS) ou `g++`;
- **git e internet**, só para a conferência com o PX4. Sem eles, rode com `SEM_PX4=1 ./rodar_tudo.sh`.

Se as ferramentas não estiverem no PATH:

```
ESBMC=/caminho/esbmc CBMC=/caminho/cbmc ./rodar_tudo.sh
```

Versões usadas até aqui e o que muda entre elas:

| | v4 (iartes, nuvem) | revisão do Prof. Lucas |
|---|---|---|
| ESBMC | 8.1.0 | 8.4.0 |
| CBMC | 5.95.1 | 6.8.0 |
| Sistema | Ubuntu 24.04, x86_64 | macOS, arm64 |

Diferenças conhecidas:

- **Resumo do `--multi-property` no ESBMC.** O 8.1 imprime `Properties: N verified … ✗ K failed`; o 8.4 imprime `** K of N properties failed`.
- **Primeira violação impressa.** No ft7, a primeira propriedade violada que o ESBMC imprime muda entre versões. O defeito é o mesmo.
- **Contagem do CBMC.** O denominador muda entre versões; as propriedades que falham são as mesmas.
- **Tamanho do READ no ASan.** Muda entre Linux e macOS (34 e 37 no bst antigo).

O veredito (FAILED ou SUCCESSFUL) de cada comando não deve mudar. Se mudar, o `RESUMO.md` marca **NAO** e o script sai com código 2.

Tempo: a maior parte termina em segundos; o mais lento é o `5_logger/esbmc` (alguns minutos). O total foi de cerca de 4 minutos (246 s) na máquina de teste (Linux, 4 núcleos, ESBMC 8.1.0, CBMC 5.95.1). Uma execução de referência dessa máquina está em `referencia_execucao_nuvem/`, só para comparação: os logs que valem são os da sua execução.

## Estrutura

```
LEIA-ME.md              este arquivo
CORRECOES_v5.md         resposta ao parecer, item a item
rodar_tudo.sh           roda tudo e gera resultados_<data>/
conferir_px4.sh         compara os trechos copiados com o PX4 (8be2590c15 e main)
candidatos/N_nome/      harness.cpp, replay.cpp e LEIA-ME.txt de cada candidato
issues_px4/             modelos das duas issues (o script gera as versões preenchidas)
documentos/             relatório ao Prof. Lucas, declaração de uso de IA,
                        guia de submissão, rascunho da issue do ESBMC
_superado_nao_usar/v4/  pacote v4 inteiro, para rastreio (não usar)
LICENSE                 MIT
```

## Depois de rodar

1. Abra o `resultados_<data>/RESUMO.md` e confira se a coluna "Confere" está toda "sim".
2. Mande a pasta `resultados_<data>/` inteira para o Prof. Lucas, junto com `documentos/relatorio_para_LC_v5.md`.
3. **Não publique nada no PX4 nem no ESBMC sem a aprovação dele.**
