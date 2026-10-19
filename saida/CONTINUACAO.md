# Continuação do trabalho: prompt para um novo assistente

Cole o bloco abaixo, inteiro, na primeira mensagem da nova sessão.

---

```
Olá! Sou a Marcela, mestranda do PPGEE/UFAM. Minha dissertação é sobre verificação
formal de módulos do autopiloto PX4 com o ESBMC (Bounded Model Checking), com
comparação com o CBMC. Estou continuando um trabalho feito com outro assistente.
Leia isto inteiro antes de agir.

## Repositório
- GitHub: marcela2881/dissertacao-px4-esbmc
- Branch de trabalho: claude/brave-euler-o09lja (trabalhe SEMPRE nela; não abra PR sem eu pedir)
- Tudo o que importa está em saida/. Leia primeiro, nesta ordem:
  1. saida/LEIA-ME_RODAR.md: passo a passo dos comandos que EU rodo
  2. saida/README.md: todos os módulos, resultados, análises e decisões
  3. saida/CONTINUACAO.md: este arquivo (estado atual e pendências)

## Regras de trabalho (importantes)
1. EU rodo todas as verificações na MINHA máquina. Você NÃO roda ESBMC/CBMC como
   resultado. Você escreve harnesses, stubs e scripts, confere a COMPILAÇÃO
   (g++ -fsyntax-only; no CBMC, no máximo --show-goto-functions), me passa os
   comandos, analisa as saídas que eu colar e registra no README.
2. Máquina oficial: "bruno-pc" (Windows + WSL Ubuntu 24.04), ESBMC 8.5.0 (Boolector)
   em ~/esbmc-8.5, CBMC 5.95.1 (apt). Pasta de trabalho: ~/verificacoes.
   Antes era a fm07 (Manchester). Todas as oficiais devem ser na MESMA máquina.
3. Para eu baixar uma versão nova, dê SEMPRE a URL do tarball pelo HASH do commit
   (o cache do GitHub para branch serve versão velha):
     cd ~/verificacoes
     curl -L -o saida.tar.gz https://github.com/marcela2881/dissertacao-px4-esbmc/archive/<HASH>.tar.gz
     tar -xzf saida.tar.gz --strip-components=2 --wildcards '*/saida/*'
4. Logs vão para o git só no final (combinado).
5. Regra 7: todo contraexemplo (FAILED) é confrontado com o código de produção
   antes de virar "achado" (é alcançável? a propriedade vem da especificação?
   é limitação da ferramenta?). Depois, replay: executar o código original com
   g++ usando os valores do contraexemplo (saida/replay/).
6. Diretrizes do PPGEE sobre LLM na dissertação: NÃO escreva texto da
   dissertação por mim, não gere referências, não direcione a pesquisa. Pode:
   revisão crítica, gramática/clareza do que EU escrevi, estrutura, brainstorming,
   explicações pontuais. Figuras/esquemas: só como apoio; eu redesenho.
7. Não ponha meus dados pessoais no git.
8. Fale comigo em português; mostre só os comandos que eu devo colar.

## Método (resumo)
- Harness = #include do .c/.cpp ORIGINAL do PX4 (commit fixado
  8be2590c1529517ccd54fbc09ae138402517baf4), stubs só do ambiente (saida/stubs),
  entradas nondet + pré-condições (__VERIFIER_assume, cada uma com origem anotada),
  propriedades como assert (vindas de especificação: datasheet, manual, protocolo).
  Um teste por propriedade (-DTEST_CHOICE=n), entrada --function harness_main nos C++.
- Testes "por indução" partem de qualquer estado que satisfaz um invariante.
- Rodada rápida: ./rodar_modulo.sh <mod> rapido (1 execução, UA, 300 s).
  Oficial: ./rodar_modulo.sh <mod> oficial (3 execuções, mediana, UA e NUA, 600 s).
  Config por módulo em saida/modulos/NN_nome/config.sh (TESTES, OPCOES, EXTRA_n,
  SO_TESTES="n" para rodar só alguns).
- Comparação: ./comparar.sh leva1|<mod>|todos rapido|oficial (mesmo harness nos dois;
  equivalência de opções no README). No CBMC, harness C++ entra por um arquivo
  gerado com #include "cbmc/prelude_cbmc.h" + #include "<harness>" (contorna a
  glibc 2.39/libstdc++ 13 e o __builtin_isfinite sem modelo; ver o prelúdio).
- 1ª leva = 12 módulos do artigo SBESC 2026 (saida/leva1, harnesses SEM alteração;
  ATENÇÃO: nesses harnesses a lógica foi EXTRAÍDA e reescrita, não incluída do PX4:
  ameaça à validade a discutir). 2ª leva = 10 módulos de saída, com código original.

## Estado em 19/10/2026
2ª leva (ESBMC 8.5, PC do Bruno):
- 01_pca9685 (T0-T4): oficial feita. T3 FAILED = restrição só documentada (>400 Hz);
  T4 FAILED = inalcançável na produção.
- 02_roboclaw (R0-R4): oficial feita. R1 FAILED = DEFEITO REAL (byte de velocidade
  128-130 para saídas 254,5-257; manual: 0-127), confirmado no replay.
  R3 FAILED = select() após write()=-1 (secundário, vai junto na issue).
  R0 = falso positivo do ESBMC (regra de C aplicada a C++17). R4 = inalcançável.
- 05_voxl_esc (T0-T8): oficial feita, tudo SUCCESSFUL.
- 06_tap_esc (T0-T4): oficial feita; confirmar no log os vereditos T1-T4
  (grep -E "^--- tap_esc_T[1-4]|run [123]:" oficial_tap.txt).
- 03_dshot (D0-D4, telemetria serial DShotTelemetry + AM32Settings): rápido feito
  (rodou junto com outra rodada, tempos inflados). T0,T1,T2,T4 SUCCESSFUL; T3 deu
  Timed out e foi refeito sem o tratador AM32 (coberto por T4). FALTA: rodar
  SO_TESTES="3" ./rodar_modulo.sh dshot rapido, e depois a oficial.
- FALTAM (harness não escrito): 04 uavcan/actuators, 07 px4io, 08 mixer_module,
  09 control_allocation, 10 control_allocator.
CBMC:
- 1ª leva rápido (19/10): concordam em Expo, MAVLink, Magnetometer, PWM, RC, IMU (FAILED
  nos dois). Timed out em Battery, EKF2, GPS. Barometer = PARSING ERROR (parser C++
  do CBMC). VTOL FAILED = falso positivo (modelo de sqrtf do CBMC). Navigator FAILED
  era o isfinite do CBMC; com o prelúdio corrigido deu SUCCESSFUL. ESBMC 8.5 dá
  PARSING ERROR no Navigator (isfinite não declarado; no 8.1 do artigo funcionava).
- Módulos C++ da 2ª leva (pca9685, roboclaw, tap_esc, dshot): CBMC não lê o
  código do PX4 (override = default, literais 11_ms, offsetof, lambda) -> PARSING
  ERROR = resultado da comparação. voxl_esc (C) o CBMC lê.
- EM ANDAMENTO no meu PC: ./comparar.sh leva1 oficial (iniciada ~07h de 19/10).
  Resultado em logs/2026-10-19_bruno-pc_comparacao_oficial/RESUMO_COMPARACAO_leva1.txt.
  Os tempos ficaram contaminados (rodei o dshot junto): repetir a oficial à noite,
  sozinha, para os tempos da dissertação.

## Próximos passos
1. Analisar a tabela da oficial da 1ª leva (eu colo) e registrar no README.
2. dshot: T3 rápido -> oficial -> comparar.sh dshot.
3. Escrever os harnesses que faltam (sugestão: mixer_module, control_allocation,
   control_allocator, depois uavcan e px4io). Mesmo padrão dos existentes.
4. Comparações oficiais dos módulos novos (comparar.sh <mod> oficial), à noite.
5. Atualizar saida/figuras/metodologia.svg (tabela de módulos) quando houver módulos novos.
6. Coisas minhas que você pode apoiar (sem escrever por mim):
   - página do manual Basicmicro (comandos 0,1,4,5: dado 0-127) e do datasheet NXP PCA9685;
   - rascunho da issue do RoboClaw (eu escrevo, você revisa a parte técnica);
     não havia issue/PR existente sobre isso no PX4 (conferido em 17/10);
   - hipótese, perguntas de pesquisa, ameaças à validade (eu escrevo, você revisa);
   - roteiro dos slides do SBESC (apresentação 27/11/2026, 11h, sessão XIII).
```

---

## Notas que não precisam ir no prompt

- Documentos da bolsa do SBESC (recibo e declaração) e o termo de autorização da SBC foram
  preenchidos na sessão anterior e enviados como arquivos na conversa; **não estão no git**
  (têm dados pessoais). Envio da bolsa: 22 a 31/10, assunto "Student Grant 2026 - Documento
  para reembolso". O termo da SBC era opcional ("Required: No") e o prazo já fechou.
- Edital 61/2026 da PROPESP: 2ª chamada até 30/10/2026, PDF único para dppropesp@ufam.edu.br.
  Já tenho a carta de anuência e a declaração de vínculo; faltam os Anexos I e II, a cópia do
  artigo e o comprovante de submissão.
