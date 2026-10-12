# MAVLink: assertion de desenrolamento com k = 10 e k = 300

Teste feito em 12/10/2026 para sustentar a frase do Cap. 2 da dissertação:
"quando a *assertion* de desenrolamento falha, o limite k não bastou".

Ferramenta: ESBMC 8.1.0 + Boolector 3.2.4, harness `resultados/MAVLink/test_mavlink_parser.cpp`,
mesmas flags da Tabela II do artigo, mas **sem** `--no-unwinding-assertions`
(ou seja, com as assertions de desenrolamento ligadas).

| k   | veredito              | propriedade violada                                   | VCCs |
|-----|-----------------------|-------------------------------------------------------|------|
| 10  | VERIFICATION FAILED   | `unwinding assertion loop 23` (`__memcpy_impl`, string.c:278) | 161  |
| 300 | VERIFICATION SUCCESSFUL | —                                                   | 3061 |

Leitura: com k = 10 a única falha é a assertion de desenrolamento do laço do
`memcpy` (o laço é mais longo que 10); não é violação de segurança de memória nem
overflow. Com k = 300 o mesmo harness passa, agora com as assertions de
desenrolamento ligadas, o que também mostra que k = 300 cobre todos os laços
desse harness.

Logs: `logs/mavlink_k10_com_unwinding_assertions.log` e
`logs/mavlink_k300_com_unwinding_assertions.log`.

## Campanha completa com assertions de desenrolamento (12/10/2026)

`rodar_com_unwinding.sh`: os 12 harnesses da Tabela II, mesmas flags do artigo
(MAVLink `--unwind 300`, Navigator `--unwind 15`, VTOL `--no-library`), mas SEM
`--no-unwinding-assertions`. ESBMC 8.1.0 + Boolector 3.2.4. Logs em
`campanha_completa/`, resumo em `campanha_completa/RESUMO.txt`.

| Módulo | Veredito | Observação |
|---|---|---|
| Barometer, Battery, EKF2, Expo, MAVLink, Magnetometer, Navigator, PWM, RC, VTOL | VERIFICATION SUCCESSFUL | nenhuma assertion de desenrolamento falha: nenhum laço excede o limite |
| IMU_BMI088 | VERIFICATION FAILED | mesma propriedade de julho (`count <= FIFO_SIZE`, candidato FIFO) |
| GPS | VERIFICATION FAILED | a primeira falha agora é `unwinding assertion loop 24` (laço do `memcpy`, mais longo que 10) |

Leitura: o resultado principal ("dez harnesses sem violação até o limite") deixa
de depender do truncamento silencioso: com as assertions ligadas, os dez passam.
No GPS, o laço do `memcpy` excede k = 10; a violação de julho continua válida no
modelo (o BMC é sound), mas a análise do GPS com k = 10 não cobre o laço inteiro.
