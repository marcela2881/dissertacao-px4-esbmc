# MicroStrain (candidato 8): não será reportado

Decisão da autora em 2026-11-12: não abrir issue no PX4 para o candidato 8;
o achado será apenas citado na dissertação.

Motivo: impacto mínimo. A escrita de um byte além de `_port[128]` no construtor
só ocorre com um caminho de dispositivo de 128 caracteres ou mais.

Estado do código: em 2026-11-12 o trecho do construtor em
`src/drivers/ins/microstrain/MicroStrain.cpp` continuava igual no `main`
(`8e94784e1dc0`, de 2026-11-11): `max_len = math::min(sizeof(_port),
strnlen(uart_port, sizeof(_port)))` seguido de `_port[max_len] = '\0'`.

`issues_px4/issue_microstrain.md` fica como rascunho não publicado. A coluna
"Issue?" da tabela em `LEIA-ME.md` ("sim") está superada por esta decisão.
