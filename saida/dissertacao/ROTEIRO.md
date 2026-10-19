# Roteiro da dissertação (para a Marcela escrever)

**Como usar:** cada seção traz (1) **base no artigo** (a seção do artigo SBESC, versão 15, de onde
partir), (2) **acrescentar**, com os tópicos e dados novos da dissertação e onde estão no repositório,
e (3) **perguntas-guia**, para você responder escrevendo. Os tópicos são **lembretes de conteúdo, não
frases para copiar**. O texto é seu (diretrizes do PPGEE sobre LLM). Escreva uma seção, mande para
revisão e siga para a próxima.

Marcadores: **[PENDENTE: …]** = resultado que ainda vai sair · ✅ = dados já fechados.

> **Atenção (diretrizes):** o texto do artigo é seu e pode ser a base, mas **reescreva e expanda** em
> vez de colar trechos. Verifique com o orientador como citar o próprio artigo publicado
> (autoplágio). Cada referência precisa de link. A seção "Declaração de uso de IA" é obrigatória
> (cap. 8).

---

## Resumo / Abstract (escrever por último)

- Problema · método (ESBMC + harness + propriedades da especificação) · alcance (12 + 10 módulos) ·
  principais achados (GPS, RoboClaw R1/R3, comparação com o CBMC) · conclusão.
- Perguntas-guia: em 5 frases, o que você fez, como, o que achou e por que importa?

---

## Cap. 1 – Introdução

**Base no artigo:** I (Introduction).

**Acrescentar:**
- Motivação mais longa: UAVs em missões críticas; firmware do PX4 (≈710k LoC); MCU sem MMU (um erro
  de memória num driver corrompe o sistema).
- **Lacuna:** drivers de produção do PX4 não verificados formalmente; testes não cobrem entradas extremas.
- **Objetivo geral e específicos** (o artigo não separa; a dissertação deve).
- **Hipótese** [você define com o orientador].
- **Perguntas de pesquisa:** RQ1–RQ3 do artigo + novas da 2ª leva. Sugestões para discutir (você decide):
  - RQ4: é viável verificar o código **original** (sem reescrever) dos drivers de saída?
  - RQ5: como ESBMC e CBMC se comparam nos mesmos harnesses?
- **Contribuições:** as do artigo + 2ª leva com código original + comparação ESBMC×CBMC + replay +
  caso RoboClaw com correção validada.
- Organização do texto.

**Perguntas-guia:** por que verificar drivers e não só a lógica de missão? O que muda da 1ª para a
2ª leva e por quê?

---

## Cap. 2 – Fundamentação teórica

**Base no artigo:** II (Background) – BMC, ESBMC, PX4.

**Acrescentar:**
- 2.1 Verificação formal × testes (onde cada um falha). Model checking; SAT/SMT.
- 2.2 BMC: equação (1) do artigo explicada termo a termo; limite k; **asserções de desdobramento
  (UA/NUA)** e o que um SUCCESSFUL significa com e sem elas.
- 2.3 ESBMC: frontend Clang, programa GOTO, execução simbólica, solvers (Boolector); checagens
  automáticas (limites, ponteiros, overflow, shift, NaN).
- 2.4 **CBMC** (novo): frontend próprio, solver MiniSat; diferença de frontends (relevante para o cap. 6).
- 2.5 PX4: NuttX, uORB (Fig. 1 do artigo), módulos; **pipeline de saída**: mixer/`MixingOutput` →
  driver → hardware; failsafe e *flight termination*.
- 2.6 **Protocolos dos drivers da 2ª leva**: I2C (PCA9685), UART Packet Serial (RoboClaw), DShot e
  telemetria, protocolos serial voxl/tap. Só o necessário para entender as propriedades.

**Perguntas-guia:** o que o BMC garante e o que não garante? Por que a escolha do frontend importa
para código C++ moderno?

---

## Cap. 3 – Trabalhos relacionados

**Base no artigo:** III (Related Work).

**Acrescentar:**
- Mais trabalhos de BMC em firmware/embarcados; comparações entre verificadores (ESBMC×CBMC, SV-COMP).
- Verificação de autopilotos (Paparazzi/Frama-C, ArduPilot, PX4 com testes por simulação).
- **Tabela comparativa** (alvo, ferramenta, código original ou modelo, achados) com o seu trabalho na última linha.
- [Referências: você busca e põe link; eu não gero referências.]

**Perguntas-guia:** o que nenhum trabalho anterior fez que você fez?

---

## Cap. 4 – Metodologia

**Base no artigo:** IV (Methodology) – 8 etapas, seleção de módulos, propriedades.

**Acrescentar:**
- 4.1 As 8 etapas (Fig. 2 do artigo) + o que mudou na 2ª leva. Figura de apoio:
  `saida/figuras/metodologia.png` (redesenhe no seu estilo).
- 4.2 **Duas formas de harness** (ponto central):
  - 1ª leva: lógica **extraída e reescrita** no harness (o próprio artigo diz "copies of source excerpts").
  - 2ª leva: `#include` do **.c/.cpp original**; stubs só do ambiente (`saida/stubs`); teste único por
    `-DTEST_CHOICE`; `--function harness_main`.
- 4.3 **Pré-condições com origem anotada** (module.yaml, chamadores, constantes) e **propriedades da
  especificação** (datasheet/manual/protocolo). Exemplo: RoboClaw RC1, manual Rev. 6.0, pp. 48–49.
- 4.4 **Testes por indução** (invariante de estado: voxl V0, tap T1, dshot D0/D3) e **divisão por
  tamanho** (voxl V1 → T1/T5–T8) para escapar de Timed out.
- 4.5 **Protocolo de execução:** rodada rápida (1 execução, UA, 300 s) → oficial (3 execuções, mediana,
  UA+NUA, 600 s); máquina única; logs com data, versão, commit e comando (`saida/scripts/run_esbmc.sh`).
- 4.6 **Regra 7 + replay:** todo FAILED é confrontado com o código de produção; replay com g++
  (`saida/replay/`); classificação (defeito real, só documentado, inalcançável, falso positivo).
- 4.7 **Comparação com o CBMC:** mesmo harness; tabela de equivalência de opções
  (`saida/README.md`); prelúdio do CBMC e o porquê de cada ajuste (`saida/cbmc/prelude_cbmc.h`).
- 4.8 **Reporte à comunidade:** issue + PR com correção validada (RoboClaw; GPS no artigo).

**Perguntas-guia:** por que mudar da reescrita para o código original? Como se garante que uma
pré-condição não esconde um defeito?

---

## Cap. 5 – Resultados

### 5.1 Configuração experimental
- Máquina oficial (PC do Bruno, WSL Ubuntu 24.04), ESBMC 8.5.0 + Boolector, CBMC 5.95.1, commit
  `8be2590`; diferença em relação ao artigo (ESBMC 8.1, i7-13700K). ✅

### 5.2 1ª leva (12 módulos) – base no artigo V-A a V-D
- Resultados do artigo (ESBMC 8.1) ✅ + reexecução com ESBMC 8.5 (Navigator dá PARSING ERROR no 8.5:
  diferença de versão) ✅ + oficial 8.5 **[PENDENTE: tabela da oficial de 19/10, repetir à noite para
  tempos limpos]**.
- Estudos de caso do artigo: GPS (Issue #26866, PR #27576, alcançável só no modelo) e IMU BMI088.

### 5.3 2ª leva (módulos de saída, código original)
Uma subseção por módulo: o que é o módulo · funções verificadas · propriedades (fonte) · tabela T0–Tn
(veredito, mediana UA/NUA) · análise de cada FAILED.
- PCA9685 ✅ (T3: restrição só documentada >400 Hz; T4: inalcançável) – `saida/README.md`.
- RoboClaw ✅ (R1 e R3 defeitos; R0 falso positivo; R4 inalcançável).
- voxl_esc ✅ (tudo SUCCESSFUL; divisão por tamanho V1).
- tap_esc ✅ [confirmar T1–T4 no log].
- dshot [PENDENTE: T3 rápido e oficial].
- mixer_module, control_allocation, control_allocator, uavcan, px4io [PENDENTE: harness].
- Tabela-resumo da 2ª leva [PENDENTE].

### 5.4 Estudo de caso: RoboClaw ✅ (completo)
- Caminho do defeito (mixer → `updateOutputs` → `sendUnsigned7Bit`), especificação (manual Rev. 6.0),
  contraexemplo, replay (128–130), R3 (`size_t` × −1), correção (patch), antes/depois (ESBMC e replay),
  falso positivo do R0 que apareceu após a correção (por que o ESBMC para no 1º erro), issue/PR
  [PENDENTE: números]. Fonte: `saida/reporte_roboclaw/RESULTADOS.md`.

### 5.5 Comparação ESBMC × CBMC
- 1ª leva: concordam em Expo, MAVLink, Magnetometer, PWM, RC, IMU; CBMC Timed out em Battery, EKF2 e
  GPS (mesmo com 600 s); Barometer = parser do CBMC; VTOL = modelo de `sqrtf` do CBMC; Navigator =
  `isfinite` sem modelo no CBMC (corrigido no prelúdio) [PENDENTE: oficial completa].
- 2ª leva: CBMC não lê o C++ do PX4 (pca9685, roboclaw, tap_esc, dshot); lê voxl_esc (C)
  [PENDENTE: comparação oficial do voxl_esc].
- Ajustes que o CBMC precisou (prelúdio) = custo de uso.

**Perguntas-guia:** para cada FAILED, por que você classificou assim? O que a comparação diz sobre
usar BMC em C++ embarcado?

---

## Cap. 6 – Discussão e ameaças à validade

**Base no artigo:** VI (Threats) e V-E (Discussion).

**Acrescentar (lista de pontos para você desenvolver):**
- Respostas às RQs (uma por parágrafo, com os números do cap. 5).
- Validade interna: 1ª leva reescrita × código original; stubs sobre/sub-aproximam; pré-condições;
  limite k e UA/NUA; um FAILED pode esconder outro (caso R0 depois da correção do R3).
- Validade de construção: propriedades vindas de especificação (manual, datasheet); propriedade
  "só documentada" (PCA9685 T3).
- Validade externa: seleção dos módulos por viabilidade de stub; uma máquina; versões das ferramentas
  (8.1 × 8.5 no Navigator).
- Ferramentas: falsos positivos de cada uma (ESBMC: R0, regra do C em C++17; CBMC: VTOL, isfinite);
  frontends.
- Custo: harness/stubs à mão; tempo de máquina; testes que precisaram ser divididos.

---

## Cap. 7 – Conclusão e trabalhos futuros

**Base no artigo:** VII.
- Retomar objetivos e RQs · contribuições · limitações · futuro (teste em hardware da RoboClaw,
  k-induction, commander/failsafe, integração contínua).

---

## Cap. 8 – Declaração de uso de IA (obrigatória pelas diretrizes)

- Escreva você: em que o Claude foi usado (harnesses, stubs e scripts; análise de logs; revisão do
  texto) e em que não foi (redação do texto). Atenção: alinhar com o que a declaração do artigo diz.
- Anexar os relatórios dos 2+ detectores de IA (abaixo de 20%).

---

## Apêndices (sugestão)
- A: comandos e configuração (`LEIA-ME_RODAR.md`, `config.sh` de cada módulo).
- B: tabelas completas com as 3 execuções.
- C: issue e PR da RoboClaw.
- D: link do repositório e do Zenodo.

---

## Ordem sugerida para escrever
1. Cap. 4 (Metodologia) – está estável e você conhece bem.
2. Seção 5.4 (RoboClaw) – completa.
3. Cap. 2 e 3 (leitura e referências).
4. Seção 5.2 (1ª leva) quando a oficial fechar.
5. Seção 5.3, módulo a módulo, conforme fecharem.
6. Cap. 6, 7, 1, Resumo (por último).
