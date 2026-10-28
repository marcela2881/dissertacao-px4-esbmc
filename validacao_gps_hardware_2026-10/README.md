# Validação do GPS em hardware (parte 1) — 27/28-10-2026

Pixhawk 6X RT rodando o firmware compilado do commit **8be2590c15** (o mesmo da
dissertação; o hash do firmware está gravado dentro de cada log `.ulg`).

- `GPS_DUMP_COMM = 1`, `SDLOG_MODE = 2`. GPS **não** conectado nesta etapa.
- Analisado com pyulog.

## Resultado (logs 6 e 7, com GPS_DUMP_COMM = 1)

| Log | Mensagens gps_dump | len mín/máx | acima de 79 bytes |
|---|---|---|---|
| log_6 (21:37) | 158 | 207 / 207 | 0 |
| log_7 (21:38) | 17 | 207 / 207 | 0 |

207 = 128 (marca "enviado ao dispositivo") + 79 bytes de dados (máximo do
buffer `data[79]`). Em 175 mensagens, **nenhuma** passou do limite.

## Interpretação
Confirma, em execução real, o que o artigo afirma e a prova por indução
(`verificacao_gps_2026-10/test_gps_prod_inducao.cpp`, SUCCESSFUL) estabelece:
o estado do contraexemplo do ESBMC (len acima da capacidade) é inalcançável no
driver de produção; o PR #27576 é endurecimento defensivo.

## Limitação
Testado só o sentido placa → GPS. O sentido GPS → placa exige um GPS u-blox
(M8N/M9N) conectado — pendente.

## Arquivos de log brutos
Os `.ulg` (log_6, log_7) e os relatórios/hashes estão na máquina da autora:
`~/evidencia_gps_hw_2026-10-27/` (WSL) e
`C:\Users\...\Documents\evidencia_gps_hw_2026-10-27\` (Windows).
A anexar ao repositório.
