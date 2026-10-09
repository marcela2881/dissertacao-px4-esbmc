# Onde estão os logs de cada resultado do paper

| Pasta ou arquivo | Conteúdo |
|---|---|
| `medianas_logs/` | 3 execuções por módulo (ESBMC 8.1.0, Boolector 3.2.4); base da Tabela II do paper. |
| `validacao_esbmc_8.1/` | GPS, MAVLink e EKF2 com Boolector e com Z3 embutido (v4.13.3), `--unwind 10`, timeout de 300 s. O Z3 não deu veredito nos três; o Boolector concluiu. |
| `resultados/` | Execuções únicas anteriores (Z3 externo, `z3 -in`). Não são as medianas da Tabela II. |
| `resultados/IMU_BMI088/resultado_temperatura_8.1.log` | Contraexemplo de temperatura (ESBMC 8.1, Boolector): `temp_msb=63`, `temp_lsb=95`, `Temp_uint11=506`. |
| `cbmc/seeded_gps/` | CBMC 5.95.1 com o harness semeado do GPS (`--unwind` 2, 3, 5 e 10). |
| `median_esbmc.sh` | Script que gera as medianas. |

## Requisito
O ESBMC precisa ter o **Boolector embutido**. A release oficial v8.1 para Ubuntu 24.04 tem; um build local sem Boolector falha com `The boolector solver has not been built into this version of ESBMC`.

## Observação sobre o Z3
O Z3 externo (`z3 -in`) concluiu vários módulos nos logs de `resultados/`. O Z3 embutido do ESBMC 8.1 (v4.13.3) não deu veredito em GPS, MAVLink e EKF2 com `--unwind 10` em 300 s. As duas interfaces se comportam de forma diferente; as medianas do paper usam o Boolector.
