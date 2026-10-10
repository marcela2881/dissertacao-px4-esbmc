# Onde estão os logs de cada resultado do paper

| Pasta ou arquivo | Conteúdo |
|---|---|
| `medianas_logs/` | 3 execuções por módulo (12 módulos, 36 logs), todas com **ESBMC 8.1.0 e Boolector 3.2.4**. É a base da Tabela II do paper (mediana de "BMC program time"; para IMU e GPS, que param na primeira violação, "Runtime decision procedure"). |
| `reexecucoes_2026-10-09/` | Execuções posteriores, feitas com outras versões do ESBMC (8.4.0 com Bitwuzla 0.9.0 nos módulos sem violação; 7.10.0 com Boolector no GPS e no IMU; Z3 no EKF2). **Não** são a base da Tabela II e não são comparáveis entre si. Mantidas por transparência. |
| `validacao_esbmc_8.1/` | GPS, MAVLink e EKF2 com Boolector e com Z3 embutido (v4.13.3), ESBMC 8.1.0, `--unwind 10`, timeout de 300 s, em outra máquina. O Z3 não deu veredito nos três; o Boolector concluiu. Os tempos desta pasta (por exemplo, GPS 130,7 s) vêm de outra máquina e não são comparáveis com a Tabela II. |
| `resultados/` | Execuções únicas anteriores (Z3 externo, `z3 -in`). Não são as medianas da Tabela II. |
| `resultados/IMU_BMI088/resultado_temperatura_8.1.log` | Contraexemplo de temperatura (ESBMC 8.1.0, Boolector): `temp_msb=63`, `temp_lsb=95`, `Temp_uint11=506`, `temperature=86.25`. |
| `cbmc/seeded_gps/` | CBMC 5.95.1 com o harness semeado do GPS (`--unwind` 2, 3, 5 e 10). |
| `cbmc/resultado_patch_gps.txt` | CBMC no harness corrigido: 0 de 7 propriedades falhas. Não há log do ESBMC para o harness corrigido. |
| `median_esbmc.sh` | Script que gera as medianas. |

## Requisito
O ESBMC precisa ter o **Boolector embutido**. A release oficial v8.1 para Ubuntu 24.04 tem; um build local sem Boolector falha com `The boolector solver has not been built into this version of ESBMC`.

## Observação sobre o Z3
O Z3 externo (`z3 -in`) concluiu vários módulos nos logs de `resultados/`. O Z3 embutido do ESBMC 8.1 (v4.13.3) não deu veredito em GPS, MAVLink e EKF2 com `--unwind 10` em 300 s. As duas interfaces se comportam de forma diferente; as medianas do paper usam o Boolector.

## Contagens de VCC
Não são tabuladas no paper. Cada execução registra `Generated N VCC(s)` no próprio log.
