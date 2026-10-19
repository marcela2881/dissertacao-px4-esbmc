# Reporte RoboClaw (R1 e R3): o que é e passo a passo

Esta pasta reúne **só** o que é preciso para reportar à comunidade PX4 os dois defeitos
encontrados no driver da RoboClaw (`src/drivers/roboclaw`, commit `8be2590`).

| Arquivo | Para que serve |
|---|---|
| `LEIA-ME.md` | este guia |
| `RASCUNHO_ISSUE_EN.md` | rascunho da issue em inglês, **para você reescrever com as suas palavras** |
| `rodar_reporte.sh` | reproduz toda a evidência na sua máquina (ESBMC + replay + reproduções mínimas) |
| `minimo_r1.cpp` | programa pequeno, sem PX4, que mostra o byte 128–130 (para colar na issue) |
| `minimo_r3.cpp` | programa pequeno, sem PX4, que mostra o −1 virando "sucesso" (para colar na issue) |

O harness do ESBMC e o replay com o código original continuam onde estavam:
`modulos/02_roboclaw/test_roboclaw.cpp` e `replay/replay_roboclaw.cpp`.

---

## 1. O que foi encontrado

### R1: byte de velocidade fora da faixa (principal)

**Caminho no código:**
1. O mixer (`MixingOutput`) entrega ao driver um valor de saída por motor.
2. `updateOutputs()` normaliza o valor: `(saída − 127) / 127` (Roboclaw.cpp:161–162), **sem limitar**.
3. `sendUnsigned7Bit()` transforma em byte: `lroundf(|valor| × 127)` (Roboclaw.cpp:291).
4. O byte vai no comando de acionamento 0, 1, 4 ou 5 (*Drive Forward/Backwards M1/M2*).

**A especificação:** nesses comandos, o byte de dados vai de **0 a 127** (manual da Basicmicro,
*RoboClaw Series User Manual*, Rev. 5.5, firmware 4.1.16+, seção "Commands 0 - 7 Compatibility
Commands", **p. 48** (comandos 0 e 1) e **p. 49** (comandos 4 e 5): *"Valid data range is 0 - 127. A value of
127 = full speed forward, 64 = about half speed forward and 0 = full stop."* Na Rev. 5 (firmware 4.1.11+) o
mesmo texto está nas pp. 40–41. Os dois PDFs são cópias de 2015/2016; se o site da Basicmicro tiver revisão
mais nova, confira a página nela antes de enviar).

**Quando acontece:**
- Na faixa normal do mixer (0 a 254), o byte fica em 0–127. O ESBMC **prova** isso (teste R2, SUCCESSFUL).
- Mas o `module.yaml` do driver permite o parâmetro de **failsafe até 257** (`failsafe: { min: 0, max: 257 }`).
- Em *flight termination*, o mixer envia o valor de failsafe **sem limitar** (mixer_module.cpp:497–501).
- Com `RBCLW_FAIL1`/`RBCLW_FAIL2` em 255, 256 ou 257, o driver envia os bytes **128, 129 ou 130**.

| saída | byte enviado |
|---|---|
| 254 | 127 (ok) |
| 255 | **128** |
| 256 | **129** |
| 257 | **130** |

**O que ainda não sabemos:** o que a RoboClaw **faz** ao receber 128–130. Só o teste no hardware
responde isso. Na issue, **não afirme** o efeito sem ter testado.

### R3: erro de `write()` tratado como sucesso (secundário, vai na mesma issue)

Em `writeCommand()` e `writeCommandWithPayload()`, o retorno de `write()` é guardado em `size_t`:

```cpp
size_t bytes_written = write(_uart_fd, buffer, 2);
if (bytes_written < 2) { return ERROR; }
return OK;
```

Pelo POSIX, `write()` devolve **−1** quando falha. Num `size_t` (sem sinal), −1 vira o maior
número possível (18446744073709551615). Assim, o teste `< 2` dá falso e a função devolve **OK**.
O driver então fica esperando uma resposta que não vai chegar. No replay, `select()` foi chamado
7 vezes depois da falha.

---

## 2. Evidências que você já tem

| Evidência | R1 | R3 |
|---|---|---|
| ESBMC 8.5 no código original (oficial, PC do Bruno, 18/10/2026) | FAILED 3/3 | FAILED 3/3 |
| ESBMC, faixa normal 0–254 (R2) | SUCCESSFUL 3/3 (o problema é só acima de 254) | — |
| Replay: g++ executa o `Roboclaw.cpp` original | 255→128, 256→129, 257→130 | `select()` 7× após `write()` = −1 |
| Reprodução mínima sem PX4 | `minimo_r1.cpp` | `minimo_r3.cpp` |
| CBMC | não lê o C++ do PX4 (`PARSING ERROR`) | idem |
| Hardware | **pendente** (opcional) | **pendente** (opcional) |

---

### Execução de referência (PC do Bruno, 19/10/2026 08:47)

`./reporte_roboclaw/rodar_reporte.sh`, ESBMC 8.5.0 + Boolector, g++ 13.3.0, PX4 `8be2590`:
- ESBMC: T1 (R1) FAILED 5.872 s; T2 (R2) SUCCESSFUL 7.546 s; T3 (R3) FAILED 54.229 s. Rodou junto
  com a oficial da 1ª leva, então os tempos não são de referência; os vereditos são.
- Replay: 254.5→128, 255→128, 256→129, 257→130 (VIOLA); 0, 127, 200, 254 ok. R3: `select()` 7 vezes
  depois de `write()` = −1. `readEncoder()` acaba devolvendo ERROR só porque a resposta simulada
  não passa no CRC, e **não** porque a falha de escrita foi detectada.
- Mínimos: iguais ao replay (128/129/130; `bytes_written = 18446744073709551615`).

## 3. Passo a passo

### Passo 1: reproduzir tudo na sua máquina (💻 PC do Bruno, ~5 min)

Só rode quando **nenhuma outra verificação** estiver rodando.

```bash
cd ~/verificacoes
curl -L -o saida.tar.gz https://github.com/marcela2881/dissertacao-px4-esbmc/archive/324709578b5e686c6a68e2f2217a2c9755f96092.tar.gz
tar -xzf saida.tar.gz --strip-components=2 --wildcards '*/saida/*'
./reporte_roboclaw/rodar_reporte.sh
```

(Se houver uma versão mais nova, troque o hash pelo commit mais recente da branch.)

**Resultado esperado:**
- **Etapa 1/3 (ESBMC):** T1 = FAILED, T2 = SUCCESSFUL, T3 = FAILED.
- **Etapa 2/3 (replay):** a tabela com os bytes 128, 129 e 130 marcados "VIOLA (> 127)" e "select() chamado 7 vez(es)".
- **Etapa 3/3 (mínimos):** `255 → 128 … 257 → 130` e `write() returned -1, writeCommand() returned OK (wrong)`.

Tudo fica salvo em `logs/<data>_bruno-pc_reporte_roboclaw/`.

### Passo 2: especificação (feito em 19/10/2026)

Encontrado no manual (Rev. 5.5, pp. 48–49; Rev. 5, pp. 40–41), já colocado no rascunho.
**Rev. 6.0 (atual, site oficial, firmware 4.3.0+), conferida em 19/10/2026:** seção "Commands 0 - 7
Compatibility Commands"; comandos 0 e 1 na **p. 48**, comando 4 na **p. 49**, com o mesmo texto
("Valid data range is 0 - 127"). O comando 5 vem logo após o 4 (na Rev. 5.5 também está na p. 49):
só confira de passagem. O rascunho já cita a Rev. 6.0, pp. 48–49.

### Passo 3: conferir se já não foi reportado (você)

1. Em github.com/PX4/PX4-Autopilot/issues, procure por `roboclaw` (abertas **e** fechadas) e veja os *pull requests* também.
2. Confira se o código ainda é igual na branch `main` (Roboclaw.cpp, linhas de `updateOutputs`,
   `sendUnsigned7Bit`, `writeCommand`). Em 17/10/2026 estava igual e não havia issue nem PR sobre isso.
3. Anote a data em que conferiu (vai no campo *Software Version*).

**Conferência feita em 19/10/2026 (pelo assistente; confira você também antes de enviar):**
- `Roboclaw.cpp`, `Roboclaw.hpp` e `module.yaml` na `main` do PX4 são **idênticos** aos do commit `8be2590`
  (comparação byte a byte). `mixer_module.cpp` mudou, mas o trecho de *termination* e `actualFailsafeValue()`
  estão iguais.
- Busca "roboclaw" no GitHub do PX4: 39 issues/PRs. Nenhum trata do byte acima de 127, da faixa do
  failsafe ou do retorno de `write()`. O único PR aberto que mexe no driver é o #28848
  ("feat(rover): add high level encoder support", aberto em 26/09/2026): ele só acrescenta 3 linhas em
  `readEncoder()` (`counts_per_rev`) e não altera `updateOutputs`, `sendUnsigned7Bit`, `writeCommand`,
  `writeCommandWithPayload` nem o `module.yaml`. Decisão: não citar esse PR na issue.

### Passo 4 (opcional, mas fortalece): hardware

Se tiver acesso a uma RoboClaw:
1. Configure `RBCLW_FAIL1 = 257`.
2. Acione o *flight termination*.
3. Observe o motor e, se possível, os bytes na UART com um analisador lógico ou um adaptador USB-serial.

Anote o que aconteceu. Sem esse teste, a issue diz apenas que o byte sai da faixa da especificação.

### Passo 5: escrever a issue com as suas palavras (você)

1. Leia o `RASCUNHO_ISSUE_EN.md`. Ele segue o modelo *Bug report* do PX4.
2. Reescreva com as suas palavras, preenchendo os `[[ ]]`.
3. Me envie a sua versão: eu confiro a parte técnica (linhas, valores, comandos, correção sugerida).

### Passo 6: enviar

1. Entre em github.com/PX4/PX4-Autopilot → **Issues** → **New issue** → **Bug report**.
2. Cole o texto. Os trechos de código entram entre três crases (```).
3. Depois de enviar, anote o link da issue no `saida/README.md` (seção RoboClaw) e na dissertação.

---

## 4. Cuidados

- Não afirme que "o motor dispara" ou algo parecido sem ter testado no hardware.
- O R1 só acontece com o **failsafe configurado acima de 254**. Diga isso claramente: é uma
  configuração **permitida** pelo próprio `module.yaml`, mas não é o padrão.
- Proponha a correção como **sugestão**: quem decide são os mantenedores.
- O R3 é secundário. Pode ir na mesma issue, como segundo ponto.
