# Verificação dos módulos de saída do PX4 (fase 2 da pesquisa)

Diferença principal em relação ao artigo anterior: **os harnesses não copiam o
código do PX4**. Cada harness é verificado junto com os arquivos **originais**
do PX4 no commit fixado, e só as dependências externas (I2C, uORB, NuttX,
log, configuração de placa) são substituídas por stubs.

## Estrutura

```
saida/
├── LEIA-ME_RODAR.md                       # passo a passo para rodar na fm07
├── rodar_modulo.sh                        # roda 1 módulo (modo rapido ou oficial)
├── rodar_tudo.sh                          # roda todos os módulos
├── modulos/<NN>_<modulo>/
│   ├── test_<modulo>.cpp|.c               # entradas não determinísticas + asserções
│   └── config.sh                          # pastas do PX4, testes e opções do ESBMC
├── stubs/                                 # substitutos das dependências externas
├── ferramenta/                            # reprodutores mínimos de problemas do ESBMC
├── scripts/get_px4.sh                     # baixa o PX4 no commit fixado
├── scripts/run_esbmc.sh                   # roda o ESBMC N vezes e registra tudo no log
├── scripts/empacotar_logs.sh              # compacta os logs para levar ao PC
└── logs/<data>_<máquina>_esbmc<versão>_<modo>/   # uma pasta por rodada
```

## Commit do PX4

`8be2590c1529517ccd54fbc09ae138402517baf4` (o mesmo do artigo anterior).

## Como rodar

Ver `LEIA-ME_RODAR.md`. Resumo:

```bash
./rodar_modulo.sh roboclaw rapido     # 1 execução por teste (achar problema de ferramenta)
./rodar_modulo.sh roboclaw oficial    # 3 execuções, com e sem asserções de desdobramento
```

Cada log começa com: data, máquina, versão do ESBMC, commit do PX4 e a linha de
comando completa, e termina com o código de saída do ESBMC.

## Regras do protocolo (lições do artigo anterior)

1. Não copiar código do PX4; usar os arquivos originais.
2. Registrar a linha de comando em todo log (o script faz isso).
3. Rodar cada teste **com** asserções de desdobramento (padrão no ESBMC 8.1/8.5) e **sem** (`--no-unwinding-assertions`) e relatar os dois.
4. Usar `__VERIFIER_assume` nos harnesses: ESBMC e CBMC aceitam, então a
   pré-condição é a mesma nas duas ferramentas.
5. Pré-condições tiradas do próprio PX4 (module.yaml, parâmetros, chamadores),
   com a origem anotada no harness.
6. Propriedades de datasheet com **seção e página** do datasheet anotadas.
7. Todo contraexemplo é confrontado com o código de produção antes de ser
   chamado de defeito.

## Status por módulo

| Módulo | Harness | Compila com o código real | ESBMC | CBMC |
|---|---|---|---|---|
| pca9685_pwm_out | `modulos/01_pca9685/` | ✅ (g++ C++17) | ✅ ESBMC 8.5.0 + Boolector na fm07 (15/10/2026) | ❌ o frontend C++ do CBMC 5.95.1 não aceita `override = default` nem `uint16_t inline` no PCA9685.h |
| roboclaw | `modulos/02_roboclaw/` | ✅ (g++ C++17) | rodada de referência pendente (fm07, ESBMC 8.5, código original) | não testado |
| voxl_esc (protocolo) | `modulos/05_voxl_esc/` | ✅ (gcc C11) | rodada de referência pendente | não testado |
| tap_esc (protocolo) | `modulos/06_tap_esc/` | ✅ (g++ C++17) | teste rápido pendente | não testado |

## RoboClaw (src/drivers/roboclaw)

Testes (`modulos/02_roboclaw/test_roboclaw.cpp`, entrada `--function harness_main`). No resumo, R0–R4 aparecem como T0–T4:

| Teste | O que verifica | Pré-condição (origem) |
|---|---|---|
| R0 | `readEncoder()`: memória, CRC, deslocamentos (`--ub-shift-check`), divisão por `RBCLW_COUNTS_REV` | endereço 128..135, counts ≥ 1 (module.yaml) |
| R1 | RC1: byte de dados dos comandos 0/1/4/5 ≤ 127 (manual Basicmicro, **página a conferir**) | outputs em [0, 257] (min/max/disarmed/failsafe do module.yaml) |
| R2 | igual a R1, só faixa normal do mixer | outputs em [0, 254] |
| R3 | se `write()` devolve -1 (POSIX), o driver não deve seguir para `select()`/`read()` | — |
| R4 | `setMotorDutyCycle()` com qualquer float (`--nan-check`); método sem chamador no PX4 | nenhuma |

**Limitações do ESBMC 8.1.0 encontradas (problemas da ferramenta, não do PX4)** — reprodutores em `ferramenta/`.
Os itens 1 e 2 **já estão corrigidos no ESBMC 8.5.0** (testado na fm07 em 16/10/2026: os dois
reprodutores dão `VERIFICATION SUCCESSFUL`), então a rodada oficial usa o código original, sem normalização.
1. `UserDefinedLiteral` não suportado (`11_ms` em `initializeUART`): `CONVERSION ERROR` no 8.1.0.
2. *Segmentation fault* na conversão quando a definição de um método usa `const float` no
   parâmetro e a declaração usa `float` (`sendUnsigned7Bit`). C++ válido.
3. O nome `main` fica ambíguo porque o código original chama `ModuleBase::main()`; o harness
   usa `harness_main` como ponto de entrada.

Com o ESBMC 8.1, os itens 1 e 2 exigiam trocar 2 linhas do `Roboclaw.cpp` (sem mudar o
significado); como o 8.5 aceita o código, essa troca não é usada (o script de normalização
saiu do repositório; está no histórico do git, commit 49cfc6c).
4. No 8.1 com Z3, `--overflow-check`/`--ub-shift-check` geram fórmula inválida no Z3 ("bit-vector
   size must be greater than zero"). A verificar no 8.5 com Boolector durante a rodada rápida.

## Prova de conceito (15/10/2026) — ESBMC 8.1.0 compilado com Z3, ambiente de nuvem

Só para validar a infraestrutura; **não são os resultados do artigo** (estes
devem ser gerados na máquina de referência, com Boolector, 3 execuções).

| Teste | Veredito | Tempo |
|---|---|---|
| T0 memória (updateRAW) | SUCCESSFUL | 0,56 s |
| T1 DS2 (LEDn_OFF_H) | SUCCESSFUL | 0,03 s |
| T2 DS3 (PRE_SCALE) | SUCCESSFUL | 9,3 s |
| T3 DS1 (pulso → contagem 12 bits) | FAILED | 140,3 s |
| T4 updateFreq sem pré-condição | FAILED | 6,6 s |

Contraexemplos (dados brutos, **ainda não analisados**):
- T3: `freq = 1524,39 Hz` (dentro da faixa do parâmetro), `divider = 3`,
  `pulse = 1024 us` → `raw = 6400` (> 4096).
- T4: `freq = -1,84e-40` (fora da faixa do parâmetro) → overflow de ponto
  flutuante na divisão em `PCA9685.cpp:117`.

Pendente (análise da pesquisadora): confrontar cada contraexemplo com o código de
produção e com a documentação do parâmetro `PCA9685_PWM_FREQ` (que diz que
frequências acima de 400 Hz só fazem sentido no modo duty-cycle).

## Execução de referência — PCA9685 (15/10/2026)

- Máquina: `fm07.cs.manchester.ac.uk`, Intel Xeon E5-2620 v4 @ 2,10 GHz, AlmaLinux 10.2
- ESBMC 8.5.0 (release oficial `esbmc-linux.zip`), solver Boolector
- PX4 commit `8be2590c1529517ccd54fbc09ae138402517baf4`
- `--unwind 70 --overflow-check --default-solver boolector --timeout 600s`, 3 execuções, mediana
- O harness inclui o `PCA9685.cpp` original (unidade de tradução única): no ESBMC 8.5,
  passar os dois `.cpp` separados gerou `main symbol 'main' not found`.

| Teste | Com asserções de desdobramento | Sem asserções de desdobramento |
|---|---|---|
| T0 memória (updateRAW) | SUCCESSFUL — 1,153 s | SUCCESSFUL — 1,148 s |
| T1 DS2 (LEDn_OFF_H ≤ 0x0F) | SUCCESSFUL — 0,047 s | SUCCESSFUL — 0,048 s |
| T2 DS3 (PRE_SCALE ≥ 3) | SUCCESSFUL — 1,147 s | SUCCESSFUL — 1,159 s |
| T3 DS1 (pulso → contagem ≤ 4096) | **FAILED** — 21,730 s | **FAILED** — 21,780 s |
| T4 updateFreq sem pré-condição | **FAILED** — 4,144 s | **FAILED** — 4,026 s |

Logs: `logs/2026-10-15_fm07_esbmc8.5.0/` (a copiar da fm07). Análise dos contraexemplos: pendente (pesquisadora).

### Teste rápido na fm07 (16/10/2026, ESBMC 8.5.0 + Boolector) — análise pendente (pesquisadora)

1ª tentativa: PARSING ERROR (`redefinition of 'iovec'`: `<fcntl.h>` do sistema traz `struct iovec`,
que o ESBMC 8.5 já define). Corrigido no stub, sem tocar no PX4.

| Teste | Veredito | Violação | Leitura preliminar (a confirmar pela pesquisadora) |
|---|---|---|---|
| R0 | FAILED | `arithmetic overflow on shl` em `swapBytesInt32` (`Roboclaw.cpp:456`) | provável falso positivo da ferramenta: regra do C aplicada a código C++17 (PX4 usa `CMAKE_CXX_STANDARD 17`), onde o deslocamento é definido |
| R1 | FAILED | byte de dados = 130 > 127 | **candidato**: contraexemplo `outputs[0] = 257` (RBCLW_FAIL permite até 257; em terminação o mixer repassa o failsafe sem limitar) |
| R2 | SUCCESSFUL | — | na faixa normal [0, 254] o byte fica ≤ 127 |
| R3 | FAILED | `select()` chamado após `write() = -1` | **candidato** (gravidade baixa): retorno de `write()` guardado em `size_t`, -1 vira SIZE_MAX e passa como sucesso |
| R4 | FAILED | NaN em `sendSigned16Bit` | sem chamador no PX4: robustez de API, não alcançável em produção |

## voxl_esc — protocolo UART (src/drivers/actuators/voxl_esc)

Harness em C (`modulos/05_voxl_esc/test_voxl_esc.c`) que inclui os originais `qc_esc_packet.c` e
`crc16.c`; não precisa de stubs. Recorte: a biblioteca de pacotes; a classe `VoxlEsc` não entra.

| Teste | O que verifica | Pré-condição (origem) |
|---|---|---|
| V0 (T0) | leitor de pacotes: de qualquer estado que satisfaz o invariante, 1 byte qualquer mantém o invariante (memória do `buffer[64]`; indução) | invariante do leitor (sobreaproximação) |
| V1 (T1, T5–T8) | P1: pacote montado por `qc_esc_create_packet` é aceito pelo leitor (formato + CRC); um tamanho por teste: T1 = 0 e 1 byte, T5 = 2, T6 = 4, T7 = 6, T8 = 10 (maior do driver) | tamanho conhecido, bytes e tipo livres |
| V2 (T2) | P2: comando de RPM enviado = pedido saturado, sem truncar nem inverter o sinal | nenhuma (qualquer int32) |
| V3 (T3) | P3: comando PWM enviado em [-800, 800], sem inverter o sinal | nenhuma (qualquer int16) |
| V4 (T4) | P4: bit de pedido de feedback só no canal `fb_id` | `fb_id` em [0, 3] (`_fb_idx` no driver) |

Teste rápido na fm07 (16/10/2026, ESBMC 8.5.0 + Boolector): V2, V3 e V4 SUCCESSFUL; V0 (70 bytes
em sequência) e V1 (payload até 57) deram *Timed out* em 300 s e foram reformulados como acima.
Segunda rodada rápida (10:00): V0 FAILED — contraexemplo mostrou erro **do invariante** (exigia
`len_expected < 63` mesmo com o leitor ocioso; o leitor rejeita o tamanho mas mantém o valor, o que é
inofensivo). Invariante corrigido -> V0 SUCCESSFUL (10:17, 0,685 s). V1 com payload até 10 e até 4: *Timed out* —
causa: tamanho simbólico fazia o ESBMC desenrolar o CRC até o limite global; V1 passou a percorrer os
tamanhos 0..10 com valores conhecidos — ainda *Timed out* (Boolector e Bitwuzla). Causa provável: P1 é
uma checagem de equivalência entre dois cálculos de CRC (montagem × leitura byte a byte), difícil para
SAT/SMT. V1 foi separado por tamanho (T1, T5–T8) para medir até onde a verificação alcança.
PC do Bruno (18/10/2026, Boolector, 1 execução): 0–1 byte 17,6 s; 2 bytes 6,6 s; 4 bytes 31,2 s; 6 bytes 37,1 s;
10 bytes *Timed out* (> 300 s). V0, V2, V3, V4 SUCCESSFUL.

## tap_esc — protocolo serial (src/drivers/tap_esc)

Harness C++ (`modulos/06_tap_esc/test_tap_esc.cpp`) que inclui o original `tap_esc_common.cpp`.
Stub novo: `stubs/fcntl.h` (o `<fcntl.h>` do sistema traz `struct iovec`, que conflita com o ESBMC 8.5).
Recorte: a biblioteca do protocolo; a classe `TAP_ESC` não entra.

| Teste | O que verifica | Pré-condição (origem) |
|---|---|---|
| T0 | TP1: `parse_tap_esc_feedback` com até 16 bytes quaisquer em qualquer posição do buffer circular (memória de `esc_feedback_buf` e `d.bytes`) | `head` < 128, `dat_cnt` ≤ 16; estado do leitor é `static` (parte de HEAD) |
| T1 | TP1: `read_data_from_uart` mantém o invariante do buffer circular a partir de qualquer estado válido (indução) | `head`, `tail`, `dat_cnt` < 128 |
| T2 | TP2: ida e volta `send_packet` → `parse_tap_esc_feedback` com `InfoRequest` (len 2) | dados livres |
| T3 | `send_packet` com qualquer `len` < 100: memória (CRC em `d.bytes[len]`) | `len` < `sizeof(d)` (remetentes usam ≤ 16) |
| T4 | TP2 com `EscbusTunePacket` (len 5) | dados livres |

Teste rápido no PC do Bruno (18/10/2026): T1 e T3 SUCCESSFUL; T0 (24 bytes), T2 e T4 *Timed out* em 300 s.
Causa: `--unwind 130` global + tamanho lido do buffer -> laço do CRC desenrolado até 130. Correção: `--unwind`
por teste (20 em T0/T2/T4), T0 com até 16 bytes e, na ida e volta, cabeçalho gravado com os valores já conferidos.
2º teste rápido e 1ª oficial (18/10): T1–T4 SUCCESSFUL; T0 FAILED em 0 s só com asserções de desdobramento =
laço de 128 voltas do PRÓPRIO harness (preencher o buffer) acima do --unwind 20 — erro do harness, não do PX4.
Corrigido (buffer sem laço de preenchimento); T0 deve ser repetido.

## Replay (teste de reprodução dos contraexemplos) — regra 7

`replay/rodar_replay.sh` compila com o **g++ comum** os arquivos ORIGINAIS do PX4 (mesmos stubs, mas com o
hardware simulado de forma determinística) e EXECUTA o driver com os valores dos contraexemplos do ESBMC.
Logs em `logs/<data>_<máquina>_replay/`.

| Candidato | O que o replay mostra (execução nativa, 18/10/2026) |
|---|---|
| RoboClaw R1 | `outputs[0]` = 254,5 / 255 / 256 / 257 → byte de dados 128 / 128 / 129 / 130 no comando DriveForwardMotor1; 0–254 → ≤ 127 |
| RoboClaw R3 | com `write()` = -1, o driver chama `select()` 7 vezes (esperado 0) e só falha depois, no CRC |
| PCA9685 T3 | 1524,39 Hz + 1024 µs → contagem 6400; `LED0_OFF_H` = 0x19 com o bit 4 (full OFF) ligado → canal desligado. Varredura com 2200 µs: ok até 450 Hz; 600 Hz e 1525 Hz desligam o canal; 1000 Hz gera pulso errado. A documentação de `PCA9685_PWM_FREQ` já proíbe > 400 Hz com pulsos (restrição só documental, não verificada no código) |

## Rodada oficial — PC do Bruno (18/10/2026)

- Máquina: `bruno-pc` (WSL), ESBMC 8.5.0, solver Boolector, PX4 `8be2590c15`
- 3 execuções por teste (mediana), com (UA) e sem (NUA) asserções de desdobramento, timeout 600 s
- Logs: `logs/2026-10-18_bruno-pc_esbmc8.5.0_oficial/` (no PC; sobem ao GitHub no final)
- Em todos os testes, as 3 execuções deram o mesmo veredito.

| Módulo | Teste | Veredito | Mediana UA (s) | Mediana NUA (s) |
|---|---|---|---|---|
| pca9685 | T0 memória (updateRAW) | SUCCESSFUL | 0,745 | 0,803 |
| pca9685 | T1 DS2 (LEDn_OFF_H) | SUCCESSFUL | 0,042 | 0,043 |
| pca9685 | T2 DS3 (PRE_SCALE) | SUCCESSFUL | 0,788 | 0,845 |
| pca9685 | T3 DS1 (contagem ≤ 4096) | FAILED | 12,435 | 12,512 |
| pca9685 | T4 updateFreq sem pré-condição | FAILED | 2,326 | 2,618 |
| roboclaw | R0 readEncoder | FAILED | 44,046 | 47,955 |
| roboclaw | R1 byte ≤ 127 (faixa com failsafe) | FAILED | 4,121 | 3,571 |
| roboclaw | R2 byte ≤ 127 (faixa normal) | SUCCESSFUL | 6,320 | 5,893 |
| roboclaw | R3 erro de write() | FAILED | 59,209 | 29,617 |
| roboclaw | R4 setMotorDutyCycle (NaN) | FAILED | 2,014 | 2,018 |
| voxl_esc | V0 leitor (invariante) | SUCCESSFUL | 0,428 | 0,557 |
| voxl_esc | V1 ida e volta, payload 0–1 | SUCCESSFUL | 22,937 | 17,425 |
| voxl_esc | V1 ida e volta, payload 2 (T5) | SUCCESSFUL | 6,537 | 6,670 |
| voxl_esc | V1 ida e volta, payload 4 (T6) | SUCCESSFUL | 34,313 | 31,391 |
| voxl_esc | V1 ida e volta, payload 6 (T7) | SUCCESSFUL | 38,321 | 39,247 |
| voxl_esc | V1 ida e volta, payload 10 (T8) | SUCCESSFUL | 238,022 | 247,535 |
| voxl_esc | V2 comando RPM | SUCCESSFUL | 0,309 | 0,299 |
| voxl_esc | V3 comando PWM | SUCCESSFUL | 0,041 | 0,042 |
| voxl_esc | V4 bit de feedback | SUCCESSFUL | 0,562 | 0,557 |

| tap_esc | T0 leitor, até 16 bytes (repetido após correção do harness) | SUCCESSFUL | 11,375 | 10,301 |
| tap_esc | T1 buffer circular (invariante) | a confirmar* | 11,392 | 15,233 |
| tap_esc | T2 ida e volta, InfoRequest (len 2) | a confirmar* | 8,968 | 8,993 |
| tap_esc | T3 send_packet, qualquer len < 100 | a confirmar* | 0,544 | 0,514 |
| tap_esc | T4 ida e volta, TunePacket (len 5) | a confirmar* | 11,422 | 10,228 |

\* medianas de `oficial_tap.txt`; vereditos SUCCESSFUL no teste rápido, a conferir nas linhas `run` da oficial.
A 1ª oficial do tap_esc T0 (FAILED em 0 s com asserções de desdobramento) foi descartada: erro do harness, corrigido e repetido.

Os vereditos repetem os da fm07 (15–16/10/2026) nos testes rodados nas duas máquinas.
A classificação de cada FAILED (defeito, falso positivo, limitação da ferramenta) é da pesquisadora.

## Fluxo de trabalho: fm07 → PC → GitHub

1. **fm07 (sem git):** rodar os testes. Cada rodada vai para
   `logs/<data>_fm07_esbmc<versão>/`, sem sobrescrever as anteriores.
2. **fm07:** `./scripts/empacotar_logs.sh` → cria `~/logs_saida_<data>.tar.gz`.
3. **PC (WSL):** copiar o arquivo:
   `scp p79642ma@fm07.cs.manchester.ac.uk:~/logs_saida_*.tar.gz ~/pesquisa-px4/`
4. **PC:** extrair dentro do clone do repositório (`saida/`) e fazer `git add`, `commit`, `push`.
