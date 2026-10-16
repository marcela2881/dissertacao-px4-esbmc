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

## tap_esc — protocolo serial (src/drivers/tap_esc)

Harness C++ (`modulos/06_tap_esc/test_tap_esc.cpp`) que inclui o original `tap_esc_common.cpp`.
Stub novo: `stubs/fcntl.h` (o `<fcntl.h>` do sistema traz `struct iovec`, que conflita com o ESBMC 8.5).
Recorte: a biblioteca do protocolo; a classe `TAP_ESC` não entra.

| Teste | O que verifica | Pré-condição (origem) |
|---|---|---|
| T0 | TP1: `parse_tap_esc_feedback` com até 24 bytes quaisquer em qualquer posição do buffer circular (memória de `esc_feedback_buf` e `d.bytes`) | `head` < 128, `dat_cnt` ≤ 24; estado do leitor é `static` (parte de HEAD) |
| T1 | TP1: `read_data_from_uart` mantém o invariante do buffer circular a partir de qualquer estado válido (indução) | `head`, `tail`, `dat_cnt` < 128 |
| T2 | TP2: ida e volta `send_packet` → `parse_tap_esc_feedback` com `InfoRequest` (len 2) | dados livres |
| T3 | `send_packet` com qualquer `len` < 100: memória (CRC em `d.bytes[len]`) | `len` < `sizeof(d)` (remetentes usam ≤ 16) |
| T4 | TP2 com `EscbusTunePacket` (len 5) | dados livres |

## Fluxo de trabalho: fm07 → PC → GitHub

1. **fm07 (sem git):** rodar os testes. Cada rodada vai para
   `logs/<data>_fm07_esbmc<versão>/`, sem sobrescrever as anteriores.
2. **fm07:** `./scripts/empacotar_logs.sh` → cria `~/logs_saida_<data>.tar.gz`.
3. **PC (WSL):** copiar o arquivo:
   `scp p79642ma@fm07.cs.manchester.ac.uk:~/logs_saida_*.tar.gz ~/pesquisa-px4/`
4. **PC:** extrair dentro do clone do repositório (`saida/`) e fazer `git add`, `commit`, `push`.
