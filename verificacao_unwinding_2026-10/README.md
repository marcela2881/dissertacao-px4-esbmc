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
