<!--
RASCUNHO do Capítulo 4 para a Marcela revisar e reescrever.
[REF: …] = referência a inserir pela autora (com link). [CONFERIR: …] = dado a confirmar.
Figuras e tabelas numeradas provisoriamente.
-->

# 4 Metodologia

Este capítulo descreve como os módulos do PX4 foram verificados. A seção 4.1 apresenta a visão geral
do processo, que estende a metodologia de oito etapas do artigo publicado no SBESC 2026
[REF: artigo SBESC 2026]. As seções seguintes detalham a seleção dos módulos (4.2), a construção dos
*harnesses* (4.3), a definição de pré-condições e propriedades (4.4), as técnicas usadas para tornar
a verificação tratável (4.5), o protocolo de execução (4.6), a análise dos contraexemplos (4.7), a
comparação com um segundo verificador (4.8) e o reporte dos achados à comunidade (4.9).

## 4.1 Visão geral

A metodologia organiza a verificação em oito etapas (Figura 4.1): (i) seleção dos módulos;
(ii) substituição das dependências de hardware e de sistema por *stubs*; (iii) especificação das
propriedades de segurança; (iv) configuração e execução do ESBMC; (v) análise dos contraexemplos;
(vi) reporte à comunidade do PX4; (vii) validação cruzada com outro verificador e reverificação de
correções; e (viii) integração contínua. As etapas (i) e (iv)–(vi) seguem o fluxo de trabalho de
Sousa et al. [REF: Sousa et al.]. As etapas (ii), (iii), (vii) e (viii) foram acrescentadas para
tratar as particularidades de firmware embarcado.

A pesquisa foi conduzida em duas levas. A **primeira leva**, publicada no SBESC 2026, cobriu doze
módulos ao longo do caminho de sensoriamento, estimação e comunicação do PX4. A **segunda leva**,
desenvolvida nesta dissertação, concentra-se nos **módulos de saída**, isto é, no código que converte
os comandos de controle em sinais enviados aos atuadores. A principal diferença metodológica entre as
levas está na forma de construir os *harnesses* (seção 4.3): na primeira, a lógica das funções foi
extraída e reescrita no próprio *harness*; na segunda, o *harness* inclui o código-fonte **original**
do PX4, sem modificações.

Todas as verificações usam o PX4-Autopilot no commit
`8be2590c1529517ccd54fbc09ae138402517baf4`, o mesmo da primeira leva, o que permite comparar os
resultados das duas.

[FIGURA 4.1: pipeline de verificação. Base: Fig. 2 do artigo e `saida/figuras/metodologia.png`.]

## 4.2 Seleção dos módulos

Os módulos foram priorizados segundo dois critérios: o impacto na segurança do voo, medido pela
posição no caminho entre sensores e atuadores, e a viabilidade de isolar o módulo por meio de
*stubs*. O segundo critério introduz um viés de seleção, discutido no Capítulo 6, porque módulos
fortemente acoplados ao escalonador ou ao hardware ficam sub-representados.

Na segunda leva, foram selecionados dez módulos de saída (Tabela 4.1). Um erro nesse trecho do
código tem efeito direto e imediato sobre os motores e servos. Além disso, esses módulos implementam
protocolos de comunicação (I2C, UART, DShot, CAN) cujos limites são definidos por documentos
externos, como manuais e *datasheets*, o que fornece especificações independentes do próprio código.

**Tabela 4.1 – Módulos da segunda leva.**

| Módulo | Pasta no PX4 (`src/`) | Recorte verificado |
|---|---|---|
| pca9685_pwm_out | `drivers/pca9685_pwm_out` | geração de PWM via I2C (`updateRAW`, `updateFreq`, `calcRawFromPulse`) |
| roboclaw | `drivers/roboclaw` | protocolo *Packet Serial* via UART (`readEncoder`, `updateOutputs`, `setMotorDutyCycle`) |
| dshot | `drivers/dshot` | telemetria serial do ESC (`DShotTelemetry`, `AM32Settings`) |
| uavcan/actuators | `drivers/uavcan` | [CONFERIR: recorte a definir] |
| voxl_esc | `drivers/actuators/voxl_esc` | protocolo de pacotes (`qc_esc_packet.c`, `crc16.c`) |
| tap_esc | `drivers/tap_esc` | protocolo serial (`tap_esc_common.cpp`) |
| px4io | `drivers/px4io` | [CONFERIR: recorte a definir] |
| mixer_module | `lib/mixer_module` | [CONFERIR: recorte a definir] |
| control_allocation | `lib/control_allocation` | [CONFERIR: recorte a definir] |
| control_allocator | `modules/control_allocator` | [CONFERIR: recorte a definir] |

## 4.3 Construção dos *harnesses*

Um *harness* de verificação é um programa que chama as funções do módulo com entradas
não determinísticas e verifica, por meio de asserções, se as propriedades esperadas se mantêm. Ele
tem três componentes: (a) um ponto de entrada que gera as entradas e chama o código verificado;
(b) *stubs* que substituem as dependências externas; e (c) as asserções que codificam as propriedades.

### 4.3.1 Primeira leva: lógica extraída

Na primeira leva, a lógica de cada função foi extraída do PX4 e reescrita dentro do *harness*,
junto com os *stubs* e as asserções, num único arquivo por módulo. Essa abordagem simplificou o
isolamento, mas tem uma limitação: o resultado vale para a reescrita, e não necessariamente para o
código de produção. Algumas constantes, por exemplo, diferiam do original (o *buffer* do GPS tinha
200 bytes no *harness* e 79 no PX4) [REF: artigo SBESC 2026, seção V-A]. Por isso, os
contraexemplos dessa leva precisaram ser confrontados manualmente com o código original.

### 4.3.2 Segunda leva: código original

Para eliminar essa limitação, os *harnesses* da segunda leva **não copiam** o código do PX4. Cada
*harness* inclui diretamente o arquivo-fonte original (`#include "Roboclaw.cpp"`, por exemplo),
formando uma única unidade de tradução. Assim, o verificador analisa exatamente o código que roda no
veículo, e um contraexemplo aponta para uma linha real do PX4.

Somente as dependências externas ao módulo são substituídas por *stubs*, reunidos na pasta
`saida/stubs` do repositório:

- funções POSIX de comunicação (`read`, `write`, `select`), modeladas de forma não determinística: a
  UART pode devolver de −1 (erro) até o número de bytes pedido, com conteúdo arbitrário;
- o barramento I2C e o *middleware* uORB (`Publication`, `Subscription`);
- o tempo (`hrt_absolute_time`, `hrt_elapsed_time`), os parâmetros (`param_get`, `ModuleParams`),
  o registro de mensagens (`PX4_ERR`, `PX4_INFO`) e as configurações de placa.

Cada *harness* contém vários testes, um por propriedade, selecionados em tempo de compilação pela
macro `TEST_CHOICE`. Assim, cada propriedade é verificada separadamente, com seu próprio tempo e
veredito. Nos *harnesses* em C++, o ponto de entrada se chama `harness_main` (opção
`--function harness_main`), porque o código original já define, por meio de `ModuleBase`, uma função
`main` própria do módulo.

## 4.4 Pré-condições e propriedades

### 4.4.1 Pré-condições

As entradas não determinísticas são restringidas por pré-condições (`__VERIFIER_assume`) que
representam apenas os valores que o código pode de fato receber em produção. Para não esconder
defeitos, **cada pré-condição tem a sua origem anotada** no *harness*. As origens admitidas são:

- os limites de parâmetros declarados no `module.yaml` do módulo;
- o modo como o próprio PX4 chama a função (por exemplo, os tamanhos de pacote usados pelo driver);
- constantes do código, como o tamanho de um *buffer*.

Por exemplo, no driver da RoboClaw, as saídas recebidas do *mixer* foram restringidas à faixa
[0, 257], pois o `module.yaml` declara `failsafe: { min: 0, max: 257 }`, e esse valor é repassado sem
limitação durante o *flight termination* (`mixer_module.cpp`, linhas 497–501). Uma pré-condição mais
estreita, como a faixa normal [0, 254], teria ocultado o defeito descrito no Capítulo 5.

### 4.4.2 Propriedades

Dois tipos de propriedades são verificados:

1. **Propriedades automáticas** do verificador: acesso fora dos limites de vetores, ponteiros
   inválidos, divisão por zero, estouro aritmético (`--overflow-check`), deslocamentos indefinidos
   (`--ub-shift-check`) e, quando aplicável, valores NaN (`--nan-check`).
2. **Propriedades de especificação**, escritas como asserções, que vêm de documentos externos ao
   código e não do próprio código, para evitar verificar apenas que o código faz o que ele faz.
   Exemplos:
   - RoboClaw: nos comandos 0, 1, 4 e 5 do modo *Packet Serial*, o byte de dados deve estar entre 0
     e 127 (*RoboClaw Series User Manual*, Rev. 6.0, pp. 48–49) [REF: manual RoboClaw];
   - PCA9685: o contador de 12 bits não pode passar de 4096, o registrador `LEDn_OFF_H` usa no
     máximo os bits definidos e `PRE_SCALE` tem valor mínimo 3 [REF: datasheet NXP PCA9685,
     CONFERIR páginas];
   - voxl_esc, tap_esc e dshot: um pacote montado pelo próprio driver deve ser aceito pelo leitor do
     protocolo (ida e volta), e os campos decodificados devem seguir o formato descrito no código.

## 4.5 Técnicas para tornar a verificação tratável

O BMC desenrola os laços até um limite *k*. Laços longos e entradas simbólicas grandes podem fazer a
verificação estourar o tempo. Três técnicas foram usadas para evitar isso sem enfraquecer as
propriedades.

**Testes por indução.** Em leitores de protocolo que processam um byte por vez, em vez de verificar
sequências longas de bytes, o teste parte de **qualquer** estado que satisfaça um invariante, processa
um byte (ou uma leitura) e verifica se o invariante continua valendo. Se vale para um passo a partir
de qualquer estado válido, vale para sequências de qualquer tamanho. Essa técnica foi usada no leitor
de pacotes do voxl_esc (V0), no *buffer* circular do tap_esc (T1) e na telemetria do dshot (D0 e D3).
Um invariante forte demais produz falsos contraexemplos: no voxl_esc, a primeira versão do invariante
falhou e foi corrigida após análise (Capítulo 5).

**Divisão por tamanho.** Quando uma propriedade de ida e volta envolve CRC sobre dados simbólicos, o
custo cresce rapidamente com o tamanho do pacote. No voxl_esc, a verificação de todos os tamanhos de 0
a 10 bytes num único teste excedeu o tempo-limite; o teste foi então dividido em um teste por tamanho
(0–1, 2, 4, 6 e 10 bytes), o que também mostra até onde a verificação alcança.

**Limite de desdobramento por teste.** O limite *k* (`--unwind`) é definido por teste, de acordo com
o maior laço daquele teste (por exemplo, 20 para pacotes curtos e 130 para o *buffer* de 128 bytes do
tap_esc), e registrado no arquivo de configuração do módulo.

## 4.6 Protocolo de execução

Cada módulo passa por duas rodadas:

1. **Rodada rápida:** uma execução por teste, com asserções de desdobramento e tempo-limite de 300 s.
   Serve para detectar problemas de *harness* ou de ferramenta antes da rodada definitiva.
2. **Rodada oficial:** três execuções por teste, com tempo-limite de 600 s, nas duas variantes: com
   asserções de desdobramento (UA) e sem elas (NUA, `--no-unwinding-assertions`). É relatada a
   mediana dos tempos.

As duas variantes têm significados diferentes. Com UA, um resultado SUCCESSFUL garante que nenhum laço
foi cortado antes do fim. Sem UA, os laços que excedem *k* são truncados em silêncio, e um SUCCESSFUL
vale apenas para os caminhos explorados.

Todas as rodadas oficiais foram executadas na mesma máquina, para que os tempos fossem comparáveis:
um computador com Windows e WSL (Ubuntu 24.04), ESBMC 8.5.0 com o *solver* Boolector e CBMC 5.95.1
[CONFERIR: processador e memória]. Cada execução gera um registro (*log*) com data, máquina, versão
do verificador, commit do PX4, linha de comando completa e código de saída. Os *scripts*
(`rodar_modulo.sh`, `run_esbmc.sh`) e os registros estão no repositório do trabalho
[REF: repositório GitHub/Zenodo].

## 4.7 Análise dos contraexemplos e *replay*

Um resultado FAILED indica que o modelo admite uma violação, mas não prova, sozinho, um defeito no
PX4. Por isso, **todo contraexemplo é confrontado com o código de produção** antes de ser chamado de
defeito. A análise responde a três perguntas:

1. O caminho do contraexemplo é alcançável no driver real, considerando quem chama a função e com
   quais valores?
2. A propriedade violada vem de uma especificação externa ou é uma suposição do *harness*?
3. A falha pode ser uma limitação da ferramenta, como uma regra da linguagem C aplicada a código C++?

Para os casos alcançáveis, é feito um ***replay***: o arquivo-fonte original é compilado com um
compilador comum (g++) e executado com os valores do contraexemplo, com a UART simulada de forma
determinística. O *replay* mostra, fora do verificador, o comportamento do código real, por exemplo o
byte efetivamente enviado ao atuador.

Com base nessa análise, cada FAILED é classificado em uma de quatro categorias: (a) **defeito
real**, alcançável em produção e contrário à especificação; (b) **restrição apenas documentada**,
quando o código aceita uma configuração que a documentação desaconselha; (c) **inalcançável em
produção**, quando a entrada do contraexemplo não ocorre no uso real; e (d) **falso positivo da
ferramenta**.

## 4.8 Comparação com um segundo verificador

Para avaliar até que ponto os resultados dependem da ferramenta, os mesmos *harnesses* foram
verificados com o CBMC [REF: CBMC]. As pré-condições, os limites de desdobramento e o tempo-limite são
os mesmos nas duas ferramentas, e as opções foram mapeadas para os equivalentes mais próximos
(Tabela 4.2).

**Tabela 4.2 – Equivalência de opções entre ESBMC 8.5 e CBMC 5.95.**

| ESBMC | CBMC |
|---|---|
| checagens padrão (limites, ponteiros, divisão por zero) | `--bounds-check --pointer-check --div-by-zero-check` |
| `--overflow-check` | `--signed-overflow-check --float-overflow-check` |
| `--ub-shift-check` | `--undefined-shift-check` |
| com asserções de desdobramento (padrão) / `--no-unwinding-assertions` | `--unwinding-assertions` / padrão |
| *solver* Boolector | MiniSat (padrão) |

No ambiente usado (glibc 2.39 e libstdc++ 13), o *frontend* C++ do CBMC não aceitou os cabeçalhos de
matemática do sistema. Para não alterar os *harnesses*, cada um foi incluído no CBMC por meio de um
arquivo de prelúdio (`cbmc/prelude_cbmc.h`), usado apenas nessa ferramenta. O prelúdio documenta cada
ajuste e o seu motivo, entre eles a modelagem de `isfinite`, que o CBMC tratava como uma função de
retorno arbitrário. Erros de leitura do código (PARSING ERROR) e de conversão são registrados como
resultado da comparação, e não como falha de verificação.

## 4.9 Reporte à comunidade e validação de correções

Os defeitos classificados como reais são reportados aos desenvolvedores do PX4 por meio de uma
*issue* no repositório oficial. Ela contém a especificação violada, o caminho no código, uma
reprodução mínima e o *replay*. Quando há uma correção proposta (*pull request*), ela é aplicada a uma
cópia do código e verificada novamente com as mesmas propriedades, para mostrar que o contraexemplo
desaparece; o *replay* também é repetido com o código corrigido. Antes do envio, confere-se que o
código ainda é o mesmo na branch principal e que não existe *issue* sobre o assunto. Na primeira leva,
esse fluxo resultou na *Issue* #26866 e no PR #27576 (GPS); na segunda, no reporte do driver da
RoboClaw (Capítulo 5) [CONFERIR: números da issue e do PR].

## 4.10 Considerações finais do capítulo

A metodologia combina verificação formal limitada sobre o código original, propriedades extraídas de
especificações externas, confirmação dos contraexemplos por execução real e comparação entre duas
ferramentas. O capítulo seguinte apresenta os resultados obtidos com ela nas duas levas.
