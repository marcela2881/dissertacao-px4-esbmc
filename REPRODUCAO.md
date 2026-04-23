# Guia de Reprodução

Este documento descreve os passos necessários para reproduzir os experimentos de verificação formal apresentados na dissertação.

## Pré-requisitos

### Sistema operacional

Os experimentos foram conduzidos em **Ubuntu 24.04 LTS** (64-bit). Outras distribuições Linux devem funcionar, mas os comandos de instalação a seguir assumem Ubuntu/Debian.

### Hardware recomendado

- Processador x86_64 (mínimo 4 núcleos)
- 16 GB de RAM ou mais (recomendado 32 GB)
- 10 GB de espaço em disco

## Instalação do ESBMC

O ESBMC pode ser obtido diretamente do repositório oficial:

```bash
# Baixar o binário pré-compilado (recomendado)
wget https://github.com/esbmc/esbmc/releases/download/v8.1.0/ESBMC-Linux.sh
chmod +x ESBMC-Linux.sh
./ESBMC-Linux.sh --skip-license --prefix=$HOME/esbmc-v8.1
```

Alternativamente, seguir as instruções de compilação em:
[https://github.com/esbmc/esbmc](https://github.com/esbmc/esbmc)

### Verificar instalação

```bash
~/esbmc-v8.1/bin/esbmc --version
```

A saída esperada deve indicar **ESBMC v8.1.0** ou superior.

## Execução dos experimentos

### Comando padrão

Para os módulos que utilizam a configuração padrão:

```bash
~/esbmc-v8.1/bin/esbmc resultados/<Modulo>/test_<modulo>.cpp \
    --unwind 10                  \
    --no-unwinding-assertions    \
    --overflow-check             \
    --default-solver boolector   \
    --timeout 120s
```

### Ajustes específicos por módulo

Dois módulos exigem ajustes do parâmetro `--unwind`:

- **MAVLink Parser** — requer `--unwind 300` para cobrir o buffer interno de 280 bytes:

```bash
~/esbmc-v8.1/bin/esbmc resultados/MAVLink/test_mavlink_parser.cpp \
    --unwind 300                 \
    --no-unwinding-assertions    \
    --overflow-check             \
    --default-solver boolector
```

- **Navigator/Mission** — requer `--unwind 15` para cobrir a iteração sobre waypoints:

```bash
~/esbmc-v8.1/bin/esbmc resultados/Navigator_Mission/test_navigator_mission.cpp \
    --unwind 15                  \
    --no-unwinding-assertions    \
    --overflow-check             \
    --default-solver boolector
```

### Módulo VTOL

O módulo VTOL requer a flag `--no-library` devido ao uso extensivo de `sinf()` e `sqrtf()`:

```bash
~/esbmc-v8.1/bin/esbmc resultados/VTOL/test_vtol_type.cpp \
    --unwind 10                  \
    --no-unwinding-assertions    \
    --overflow-check             \
    --no-library                 \
    --default-solver boolector
```

## Interpretação dos resultados

Três saídas são possíveis:

- **`VERIFICATION SUCCESSFUL`** — nenhuma violação encontrada até a profundidade analisada. Indica que o módulo está livre das classes de vulnerabilidades verificadas.

- **`VERIFICATION FAILED`** — o ESBMC encontrou uma violação e gerou um contraexemplo. Neste caso, o log apresenta valores concretos de entrada que levam à violação. Os módulos **GPS** e **IMU_BMI088** deste repositório apresentam este resultado e correspondem às vulnerabilidades reportadas nas Issues #26866 e #26865.

- **`ERROR: Timed out`** — a verificação excedeu o tempo limite especificado. Aumentar o valor de `--timeout` ou tentar outro solver (ex: `--default-solver z3`).

## Solvers alternativos

Para os módulos que fazem uso intensivo de operações de ponto flutuante, pode ser necessário usar o solver Z3 em vez do Boolector:

```bash
--default-solver z3
```

Ou via interface SMTLIB (requer Z3 instalado separadamente):

```bash
--smtlib-solver-prog "z3 -in"
```

## Estrutura dos arquivos

Cada subpasta em `resultados/` contém:

- **`test_<modulo>.cpp`** — harness de verificação escrito em C++. Contém:
  - Stubs das dependências do módulo original do PX4
  - Declarações de entradas não-determinísticas (`nondet_*()`)
  - Assumptions que restringem o domínio das entradas (`__ESBMC_assume`)
  - Assertions que expressam as propriedades de segurança verificadas (`assert`)

- **`resultado_<modulo>.txt`** — saída completa da execução do ESBMC correspondente, incluindo:
  - Parâmetros utilizados
  - Métricas de verificação (VCCs, tempo, solver)
  - Contraexemplo completo, quando aplicável (módulos GPS e IMU)

## Referência aos módulos do PX4

Cada harness inclui, em seu cabeçalho, a indicação do arquivo original do PX4 v1.16 que motivou sua escrita. Exemplos:

- `test_bmi088_imu.cpp` → `src/drivers/imu/bosch/bmi088/`
- `test_gps_ublox.cpp` → `src/drivers/gps/`
- `test_mavlink_parser.cpp` → `src/modules/mavlink/mavlink_main.cpp`
- `test_ekf2_covariance.cpp` → `src/modules/ekf2/EKF/covariance.cpp`
- `test_vtol_type.cpp` → `src/modules/vtol_att_control/vtol_type.cpp`

Para inspecionar o código original: [https://github.com/PX4/PX4-Autopilot/tree/v1.16.0](https://github.com/PX4/PX4-Autopilot/tree/v1.16.0)

## Dúvidas e contato

Para dúvidas sobre a reprodução dos experimentos, entrar em contato pelo GitHub ([@marcela2881](https://github.com/marcela2881)) ou abrindo uma *Issue* neste repositório.
