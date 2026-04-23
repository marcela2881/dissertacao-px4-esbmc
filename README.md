# Verificação Formal de Módulos Críticos do PX4 Autopilot usando ESBMC

Repositório de artefatos da dissertação de mestrado **"Verificação Formal de Módulos Críticos do PX4 Autopilot usando ESBMC"**, desenvolvida no Programa de Pós-Graduação em Engenharia Elétrica (PPGEE) da Universidade Federal do Amazonas (UFAM), dentro do projeto SAGRES de monitoramento ambiental na Amazônia.

**Autora:** Marcela Alves Rodrigues
**Orientador:** Prof. Dr. Lucas Carvalho Cordeiro
**Instituição:** Universidade Federal do Amazonas (UFAM) — PPGEE
**Ano:** 2026

---

## Sobre o trabalho

Este trabalho aplica *Bounded Model Checking* (BMC) ao firmware do PX4 Autopilot v1.16, utilizando o verificador formal [ESBMC](https://github.com/esbmc/esbmc) para identificar vulnerabilidades de segurança em módulos críticos de controle de voo.

Foram verificados **12 módulos** do PX4 organizados em três remessas, com um total aproximado de 1.219 condições de verificação (VCCs) analisadas.

## Vulnerabilidades encontradas

A campanha de verificação identificou **quatro vulnerabilidades reais** em dois módulos da primeira remessa, reportadas oficialmente ao projeto PX4:

- **Issue #26865** — IMU BMI088: [https://github.com/PX4/PX4-Autopilot/issues/26865](https://github.com/PX4/PX4-Autopilot/issues/26865)
  - Temperatura fora da faixa operacional (sem validação)
  - Overflow do contador de FIFO (1.536 bytes excede buffer de 1.024)
- **Issue #26866** — GPS uBlox: [https://github.com/PX4/PX4-Autopilot/issues/26866](https://github.com/PX4/PX4-Autopilot/issues/26866)
  - Underflow aritmético em `dumpGpsData()`
  - Acesso à memória fora dos limites em `memcpy`

## Estrutura do repositório

```
.
├── README.md                   (este arquivo)
├── REPRODUCAO.md               (como reproduzir a verificação)
├── .gitignore
└── resultados/
    ├── Barometer/
    │   ├── test_barometer.cpp          (harness de verificação)
    │   └── resultado_barometer.txt     (saída do ESBMC)
    ├── Battery_Monitor/
    ├── EKF2_Covariance/
    ├── Expo_RC/
    ├── GPS/                            (contém bug real — Issue #26866)
    ├── IMU_BMI088/                     (contém bugs reais — Issue #26865)
    ├── Magnetometer/
    ├── MAVLink/
    ├── Navigator_Mission/
    ├── PWM_Output/
    ├── RC_Update/
    └── VTOL/
```

Cada subpasta contém **dois arquivos**:

- `test_<modulo>.cpp` — harness de verificação com stubs, assumptions e assertions
- `resultado_<modulo>.txt` — saída completa do ESBMC (incluindo contraexemplos nos casos de bug)

## Ambiente utilizado

| Componente | Versão |
|---|---|
| Ferramenta de verificação | ESBMC v8.1.0 (64-bit, x86_64 Linux) |
| Solver padrão | Boolector 3.2.4 |
| Solver alternativo | Z3 (via interface SMTLIB) |
| Sistema operacional | Ubuntu 24.04 LTS |
| Hardware | AMD Ryzen 7 5700U · 32 GB RAM DDR4 |
| Linguagem alvo | C/C++17 |
| Firmware alvo | PX4 Autopilot v1.16 |

## Como reproduzir

Consulte o arquivo [REPRODUCAO.md](REPRODUCAO.md) para instruções detalhadas de instalação do ESBMC e execução dos experimentos.

## Resumo dos resultados

| Remessa | Módulo | LOC | VCCs | Bugs | Tempo |
|---|---|---:|---:|---:|---:|
| 1ª | IMU BMI088 | 1.200 | — | 2 | — |
| 1ª | GPS uBlox | 800 | — | 2 | 12,1 s |
| 1ª | MAVLink Parser | 600 | 161 | 0 | 129,5 s |
| 1ª | Expo RC | 150 | 325 | 0 | 127,8 s |
| 2ª | Battery Monitor | — | 78 | 0 | 6,4 s |
| 2ª | PWM Output | — | 58 | 0 | 0,1 s |
| 2ª | RC Update | — | 91 | 0 | 38,9 s |
| 2ª | Navigator/Mission | — | 68 | 0 | 0,1 s |
| 3ª | EKF2 Covariance | — | 96 | 0 | 158,0 s |
| 3ª | Magnetômetro | — | 82 | 0 | 16,3 s |
| 3ª | Barômetro | — | 117 | 0 | 0,4 s |
| 3ª | VTOL | — | 143 | 0 | 10,6 s |
| **Total** | | **2.750+** | **1.219** | **4** | **~490 s** |

## Ferramentas e referências

- **PX4 Autopilot:** [https://github.com/PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) (v1.16)
- **ESBMC:** [https://github.com/esbmc/esbmc](https://github.com/esbmc/esbmc) (v8.1.0)
- **Projeto SAGRES:** monitoramento ambiental com drones na Amazônia
- **UFAM PPGEE:** [https://www.ppgee.ufam.edu.br](https://www.ppgee.ufam.edu.br)

## Contato

- Marcela Alves Rodrigues — GitHub: [@marcela2881](https://github.com/marcela2881)
- Prof. Lucas Carvalho Cordeiro — [SSVLab, University of Manchester](https://ssvlab.github.io/lucasccordeiro/)

## Licença

Os artefatos (harnesses `.cpp` e logs) deste repositório são disponibilizados para fins acadêmicos e de reprodutibilidade da pesquisa. O código-fonte do PX4 Autopilot e do ESBMC segue suas próprias licenças (BSD-3-Clause e Apache-2.0, respectivamente).
