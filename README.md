# Verificação Formal de Módulos Críticos do PX4 Autopilot usando ESBMC

Repositório de artefatos da dissertação de mestrado desenvolvida no Programa de Pós-Graduação
em Engenharia Elétrica (PPGEE) da Universidade Federal do Amazonas (UFAM), no âmbito do
projeto SAGRES de monitoramento ambiental na Amazônia.

**Autora:** Marcela Alves Rodrigues
**Orientador:** Prof. Dr. Lucas Carvalho Cordeiro (University of Manchester / UFAM)
**Coorientador:** Prof. Dr. José Reginaldo Hughes de Carvalho (UFAM)
**Instituição:** Universidade Federal do Amazonas — PPGEE · **Ano:** 2026

**DOI do artefato:** [10.5281/zenodo.21401959](https://doi.org/10.5281/zenodo.21401959)

---

## Sobre o trabalho

Este trabalho propõe e avalia uma metodologia de verificação formal para firmware de sistemas
críticos, baseada em *Bounded Model Checking* (BMC) com o verificador ESBMC. O PX4 Autopilot
v1.16 é utilizado como estudo de caso, com 12 módulos verificados e aproximadamente 1.219
condições de verificação (VCCs) analisadas.

## Resultados

**Resultado principal.** Dez dos doze módulos, cobrindo as principais classes de entrada não
confiável — barramentos seriais de sensores, enlace de rádio e telemetria, atuação, estimação
e lógica de missão — verificam-se livres de violações de segurança de memória e de *overflow*
até o limite de desenrolamento.

**Resultado secundário.** A campanha produziu três contraexemplos, que a triagem resolveu de
três formas distintas:

| Achado | Módulo | Classificação | Desfecho |
|---|---|---|---|
| *Underflow* sem sinal em `dumpGpsData()` | GPS uBlox | CWE-191 → CWE-119 | **Violação alcançável no modelo.** Reportada na [Issue #26866](https://github.com/PX4/PX4-Autopilot/issues/26866); guarda acolhida upstream como endurecimento defensivo (PR #27576). Alcançabilidade no alvo em aberto. |
| Contador do FIFO (`count = 8704`) | IMU BMI088 | CWE-190 / CWE-787 | **Falso positivo.** Refutado no *replay* contra o driver de produção não instrumentado, que protege a leitura em três níveis. O candidato surgiu porque o *harness* omitia essas guardas. |
| Temperatura fora da faixa (86,25 °C) | IMU BMI088 | CWE-20 | **Recusado upstream.** [Issue #26865](https://github.com/PX4/PX4-Autopilot/issues/26865) e PR #27575 fechados pelos mantenedores. Mantido como nota de projeto, sem consequência de segurança confirmada. |

> **Nota metodológica.** Os achados são **violações alcançáveis no modelo**, não defeitos
> confirmados em produção. Um contraexemplo estabelece alcançabilidade dentro do modelo
> analisado; estabelecer alcançabilidade *on-target* exigiria demonstrar que os valores podem
> ser produzidos pelo hardware e pela pilha de software reais. Por isso a triagem e a revisão
> pela comunidade são partes integrantes da metodologia, e não etapas acessórias.

## Estrutura do repositório

```
.
├── README.md            (este arquivo)
├── REPRODUCAO.md        (instruções de instalação e execução)
├── LICENSE
├── resultados/          (harnesses e logs do ESBMC, por módulo)
│   ├── Barometer/           test_barometer.cpp · resultado_barometer.txt
│   ├── Battery_Monitor/
│   ├── EKF2_Covariance/
│   ├── Expo_RC/
│   ├── GPS/                 violação alcançável no modelo — Issue #26866
│   ├── IMU_BMI088/          falso positivo (FIFO) + observação recusada (temperatura)
│   ├── Magnetometer/
│   ├── MAVLink/
│   ├── Navigator_Mission/
│   ├── PWM_Output/
│   ├── RC_Update/
│   └── VTOL/
└── cbmc/                (harnesses CBMC, verificação do patch e checagem no ArduPilot)
```

Cada subpasta de `resultados/` contém o *harness* de verificação, com *stubs*, *assumptions*
e *assertions*, e a saída completa do ESBMC, incluindo o contraexemplo nos casos de violação.

## Ambiente de verificação

| Componente | Versão |
|---|---|
| Verificador (campanha consolidada) | ESBMC v8.1.0 (64-bit, x86_64 Linux) |
| Verificador (achados originais) | ESBMC v7.10.0 |
| Solver padrão | Z3 v4.8.12 |
| Solver do GPS e do IMU | Boolector 3.2.4 |
| Verificação cruzada | CBMC 5.95.1 |
| Sistema operacional | Ubuntu 24.04 LTS |
| Hardware | Intel Core i7-13700K · 64 GB RAM |
| Linguagem alvo | C/C++17 |
| Firmware alvo | PX4 Autopilot v1.16 (*commit* `8be2590c15`, 04/06/2026) |

Os contraexemplos do BMI088 e do GPS foram originalmente reportados sob ESBMC v7.10.0
(Issues #26865 e #26866); os tempos abaixo foram remedidos sob a configuração da campanha em
v8.1.0.

## Resumo dos resultados

Tempos reportados como **mediana de três execuções**.

| Módulo | Desfecho | VCCs | Unwind | Solver | Tempo |
|---|---|---|---|---|---|
| IMU BMI088 | violação | n/a | 10 | Boolector | n/a |
| GPS uBlox | violação | n/a | 10 | Boolector | 64,9 s |
| MAVLink Parser | sem violação | 161 | 300 | Z3 | 49,2 s |
| Expo RC | sem violação | 325 | 10 | Z3 | 4,9 s |
| Battery Monitor | sem violação | 78 | 10 | Z3 | 6,7 s |
| PWM Output | sem violação | 58 | 10 | Z3 | 0,2 s |
| RC Update | sem violação | 91 | 10 | Z3 | 9,5 s |
| Navigator/Mission | sem violação | 68 | 15 | Z3 | 0,1 s |
| EKF2 Covariance | sem violação | 96 | 10 | Z3 | 12,2 s |
| Magnetômetro | sem violação | 82 | 10 | Z3 | 6,6 s |
| Barômetro | sem violação | 117 | 10 | Z3 | 0,4 s |
| VTOL | sem violação | 143 | 10 | Z3 | 4,0 s |
| **Total** | **10 sem violação, 2 com violação** | **1.219** | — | — | **≈160 s** |

As contagens de VCC excluem IMU e GPS, que interrompem a execução na primeira violação, e
misturam limites de desenrolamento distintos — não são diretamente comparáveis entre si. O
VTOL foi verificado com `--no-library`, o que reduz as VCCs de 17.611 para 143 e, portanto,
cobre proporcionalmente menos propriedades.

## Verificação cruzada e do *patch*

O CBMC 5.95.1 foi executado sobre os mesmos *harnesses*, com opções equivalentes. Para o GPS,
ambas as ferramentas sinalizam a violação, mas a reportam como propriedades formalmente
distintas: o ESBMC como *underflow* de inteiro sem sinal (CWE-191) e o CBMC como escrita fora
dos limites em `memcpy` (CWE-787). São as duas faces do mesmo defeito, e a divergência decorre
do tratamento da conversão com e sem sinal na subtração. **Isso não constitui confirmação
independente de alcançabilidade**: sob a mesma pré-condição, a concordância corrobora apenas a
codificação do problema.

Sobre o *patch*, o *harness* sem correção levanta sete violações de VCC ao longo do caminho do
*underflow*. Aplicada a correção, o CBMC reporta 0/7 e o ESBMC com Boolector reporta
`VERIFICATION SUCCESSFUL`. A guarda rejeita a condição exercitada pelo *harness*, não a
ausência de outros caminhos além do limite.

## Como reproduzir

Consulte [`REPRODUCAO.md`](REPRODUCAO.md).

## Publicação associada

Artigo submetido ao XVI Simpósio Brasileiro de Engenharia de Sistemas Computacionais
(SBESC 2026), trilha de artigo completo — *Formal Verification of Critical Modules in the PX4
Autopilot using Bounded Model Checking*. Em avaliação.

## Referências

- PX4 Autopilot — https://github.com/PX4/PX4-Autopilot (v1.16)
- ESBMC — https://github.com/esbmc/esbmc
- CBMC — https://github.com/diffblue/cbmc
- *Fork* com integração contínua — https://github.com/marcela2881/PX4-Autopilot
- PPGEE/UFAM — https://www.ppgee.ufam.edu.br

## Licença

Os artefatos deste repositório (*harnesses* `.cpp` e logs) são disponibilizados sob licença MIT
para fins acadêmicos e de reprodutibilidade. O PX4 Autopilot e o ESBMC seguem suas próprias
licenças, BSD-3-Clause e Apache-2.0 respectivamente.

## Contato

Marcela Alves Rodrigues — [@marcela2881](https://github.com/marcela2881)
Prof. Dr. Lucas Carvalho Cordeiro — SSVLab, University of Manchester
