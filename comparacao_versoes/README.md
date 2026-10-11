# Comparação de versões do ESBMC: 8.1.0 (julho) × 8.1.0 (nuvem) × 8.5.0 (nuvem)

Data: 11/10/2026. Os dois binários da nuvem foram compilados das tags oficiais
`v8.1` e `v8.5` com Boolector 3.2.4 e clang/LLVM 18. Os harnesses e as flags são
os da Tabela II do artigo (`comparar_versoes.sh`). Só o veredito, as VCCs e o
contraexemplo são comparáveis; **os tempos da nuvem não são**.

| Módulo | Julho 8.1.0 (iartes) | 8.1.0 nuvem | 8.5.0 nuvem |
|---|---|---|---|
| Barometer | SUCCESSFUL, 93 | SUCCESSFUL, 93 | SUCCESSFUL, 93 |
| Battery | SUCCESSFUL, 49 | SUCCESSFUL, 49 | SUCCESSFUL, 49 |
| EKF2 | SUCCESSFUL, 63 | SUCCESSFUL, 63 | SUCCESSFUL, 63 |
| Expo | SUCCESSFUL, 278 | SUCCESSFUL, 278 | SUCCESSFUL, 278 |
| GPS | FAILED, 4463; `input_len=10`, `len=207` | FAILED, 4463; 10/207 (*) | FAILED, 4263; `input_len=1`, `len=224` |
| IMU BMI088 | FAILED, 120; `count=9088` | FAILED, 120; 9088 | FAILED, 122; `count=9215` |
| MAVLink | SUCCESSFUL, 3059 | SUCCESSFUL, 3059 | SUCCESSFUL, 3059 |
| Magnetometer | SUCCESSFUL, 50 | SUCCESSFUL, 50 | SUCCESSFUL, 50 |
| Navigator | SUCCESSFUL, 50 | SUCCESSFUL, 50 | erro de *parsing* (**) |
| PWM | SUCCESSFUL, 38 | SUCCESSFUL, 38 | SUCCESSFUL, 38 |
| RC | SUCCESSFUL, 56 | SUCCESSFUL, 56 | SUCCESSFUL, 56 |
| VTOL (`--no-library`) | SUCCESSFUL, 110 | SUCCESSFUL, 110 | SUCCESSFUL, 110 |

(*) Com o `--timeout 120s` do artigo, o GPS fica sem veredito na nuvem (o
iartes leva 65 s; a nuvem leva 137 s). Com `--timeout 600s`: FAILED, 4463 VCCs,
10/207 (`logs/v8.1_nuvem/GPS_timeout600.log`).

(**) O front-end C++ da 8.5 não declara `isfinite` no escopo global
(`use of undeclared identifier 'isfinite'; did you mean 'std::isfinite'?`).
Numa cópia do harness com só `#include <cmath>` e `using std::isfinite;`
acrescentados (`navigator_v85_isfinite.cpp`), o resultado é SUCCESSFUL com 50
VCCs, igual a julho (`logs/v8.5_nuvem/Navigator_isfinite.log`). O harness
oficial não foi alterado.

## Leitura

- **8.1.0 da nuvem × julho:** 12 de 12 idênticos (veredito, VCC e contraexemplo).
  O build da nuvem é equivalente ao usado no artigo.
- **8.5.0 × julho:** os 12 vereditos se mantêm; 9 de 12 têm as mesmas VCCs.
  - **GPS** e **IMU:** mesmo teste (`test_choice` 3 e 2), mesma propriedade
    violada (acesso fora dos limites no `memcpy`; `count <= FIFO_SIZE`) e
    contagem de VCC um pouco diferente. A **testemunha** muda (GPS `1/224`, IMU
    `9215`), o que é esperado com entradas não determinísticas: o solver
    escolhe qualquer valor que viole a propriedade.
  - **Navigator:** é incompatibilidade de front-end, não diferença de
    verificação (ver acima).
- A 8.5 passa a imprimir a classificação CWE da violação. No GPS:
  `CWE: CWE-125, CWE-787, CWE-823`.
