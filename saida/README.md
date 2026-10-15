# Verificação dos módulos de saída do PX4 (fase 2 da pesquisa)

Diferença principal em relação ao artigo anterior: **os harnesses não copiam o
código do PX4**. Cada harness é verificado junto com os arquivos **originais**
do PX4 no commit fixado, e só as dependências externas (I2C, uORB, NuttX,
log, configuração de placa) são substituídas por stubs.

## Estrutura

```
saida/
├── harnesses/<modulo>/test_<modulo>.cpp   # entradas não determinísticas + asserções
├── stubs/                                 # substitutos das dependências externas
├── scripts/get_px4.sh                     # baixa o PX4 no commit fixado
├── scripts/run_esbmc.sh                   # roda o ESBMC 3x e registra tudo no log
└── logs/                                  # um log por execução
```

## Commit do PX4

`8be2590c1529517ccd54fbc09ae138402517baf4` (o mesmo do artigo anterior).

## Como rodar

```bash
cd saida/scripts
./get_px4.sh ../px4                       # uma vez
export PX4_ROOT=../px4

# PCA9685, teste 1 (DS2), com asserções de desdobramento:
./run_esbmc.sh pca9685_T1_ua ../harnesses/pca9685/test_pca9685.cpp \
  "" "src/drivers/pca9685_pwm_out" \
  -DTEST_CHOICE=1 --unwind 20 --unwinding-assertions --overflow-check \
  --default-solver boolector --timeout 300s
```

Cada log começa com: data, máquina, versão do ESBMC, commit do PX4 e a linha de
comando completa.

## Regras do protocolo (lições do artigo anterior)

1. Não copiar código do PX4; usar os arquivos originais.
2. Registrar a linha de comando em todo log (o script faz isso).
3. Rodar cada teste **com** asserções de desdobramento (padrão no ESBMC 8.1) e **sem** (`--no-unwinding-assertions`) e relatar os dois.
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
| pca9685_pwm_out | `harnesses/pca9685/` | ✅ (g++ C++17) | ✅ aceita o código real (prova de conceito com Z3, logs `logs/poc_*`) | ❌ o frontend C++ do CBMC 5.95.1 não aceita `override = default` nem `uint16_t inline` no PCA9685.h |

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
