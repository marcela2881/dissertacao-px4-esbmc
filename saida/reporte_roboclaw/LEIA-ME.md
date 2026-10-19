# Reporte RoboClaw (R1 e R3): o que é e passo a passo

Esta pasta reúne **tudo e só** o que é preciso para reportar à comunidade PX4 os dois defeitos
encontrados no driver da RoboClaw (`src/drivers/roboclaw`, commit `8be2590`) e propor a correção.

```
reporte_roboclaw/
├── LEIA-ME.md                     este guia
├── rodar_reporte.sh               reproduz TODA a evidência (original e corrigido) na sua máquina
├── 1_verificacao_esbmc/
│   └── test_roboclaw.cpp          harness do ESBMC (inclui o Roboclaw.cpp ORIGINAL); T1=R1, T2=R2, T3=R3
├── 2_replay/
│   └── replay_roboclaw.cpp        "teste unitário": g++ executa o driver com os valores do contraexemplo
├── 3_minimos/
│   ├── minimo_r1.cpp              8 linhas, sem PX4: mostra os bytes 128–130 (vai na issue)
│   └── minimo_r3.cpp              sem PX4: mostra o −1 virando "sucesso"
├── 4_manual/
│   └── MANUAL.md                  especificação: manual da RoboClaw, Rev. 6.0, pp. 48–49
├── 5_issue/
│   ├── RASCUNHO_ISSUE_EN.md       rascunho da issue (inglês) para você reescrever
│   └── RASCUNHO_ISSUE_PT.md       tradução, só para entender
└── 6_pr/
    ├── correcao_roboclaw.patch    a correção proposta (3 trechos do Roboclaw.cpp)
    └── RASCUNHO_PR_EN.md          rascunho do pull request para você reescrever
```

O harness e o replay são **cópias** (19/10/2026) de `modulos/02_roboclaw/` e `replay/`, guardadas aqui
para a pasta ficar completa.

---

## 1. O que foi encontrado

### R1: byte de velocidade fora da faixa (principal)

**Caminho no código:**
1. O mixer (`MixingOutput`) entrega ao driver um valor de saída por motor.
2. `updateOutputs()` normaliza esse valor com `(saída − 127) / 127`, **sem limitar** (Roboclaw.cpp:161–162).
3. `sendUnsigned7Bit()` transforma o resultado em byte com `lroundf(|valor| × 127)` (Roboclaw.cpp:291).
4. O byte vai no comando de acionamento 0, 1, 4 ou 5 (*Drive Forward/Backwards M1/M2*).

**A especificação:** nesses comandos, o dado vai de **0 a 127**. O manual da RoboClaw diz, na Rev. 6.0,
pp. 48–49: *"Valid data range is 0 - 127"* (detalhes em `4_manual/MANUAL.md`).

**Quando acontece:**
- Na faixa normal do mixer (0 a 254), o byte fica em 0–127. O ESBMC **prova** isso (R2, SUCCESSFUL).
- Mas o `module.yaml` do driver permite **failsafe até 257** (`failsafe: { min: 0, max: 257 }`).
- No *flight termination*, o mixer envia o failsafe **sem limitar** (mixer_module.cpp:497–501).

| `RBCLW_FAILn` | byte enviado |
|---|---|
| 254 | 127 (ok) |
| 255 | **128** |
| 256 | **129** |
| 257 | **130** |

**O que não sabemos:** o que a RoboClaw **faz** com 128–130. Só um teste no hardware responde isso.

### R3: erro de `write()` tratado como sucesso (secundário, vai na mesma issue)

Em `writeCommand()` e `writeCommandWithPayload()`, o retorno de `write()` é guardado em `size_t`
(inteiro sem sinal). Quando `write()` falha, devolve **−1**. Num `size_t`, isso vira
18446744073709551615, então o teste `< 2` dá falso e a função devolve **OK**. O driver fica esperando
uma resposta que não vai chegar. No replay, `select()` foi chamado 7 vezes depois da falha. O
`readEncoder()` só acaba em ERROR porque a resposta simulada falha no CRC, **não** porque a falha de
escrita foi detectada.

### A correção proposta (`6_pr/correcao_roboclaw.patch`)

| Onde | Antes | Depois |
|---|---|---|
| `sendUnsigned7Bit()` | `lroundf(fabs(data) * INT8_MAX)` | `lroundf(math::constrain(fabsf(data), 0.f, 1.f) * INT8_MAX)` |
| `writeCommandWithPayload()` | `size_t bytes_written = write(...)` | `ssize_t bytes_written = write(...)`; compara com `static_cast<ssize_t>(packet_size)` |
| `writeCommand()` | `size_t bytes_written = write(...)` | `ssize_t bytes_written = write(...)` |

A correção também ajusta duas mensagens de log: troca `%zu` por `%zd` e passa a mostrar o tamanho
do pacote. O `module.yaml` **não** é alterado; o PR menciona isso como alternativa.

---

## 2. Evidências

| Evidência | R1 | R3 |
|---|---|---|
| ESBMC 8.5 no original (oficial, PC do Bruno, 18/10/2026) | FAILED 3/3 | FAILED 3/3 |
| ESBMC, faixa normal 0–254 (R2) | SUCCESSFUL 3/3 | — |
| Replay com o `Roboclaw.cpp` original (19/10/2026) | 255→128, 256→129, 257→130 | `select()` 7× após `write()` = −1 |
| Reprodução mínima sem PX4 | `minimo_r1.cpp` | `minimo_r3.cpp` |
| Especificação | manual Rev. 6.0, pp. 48–49 | POSIX: `write()` devolve −1 em erro |
| **ESBMC com a correção** | **a rodar (passo 1)**, esperado SUCCESSFUL | **a rodar**, esperado SUCCESSFUL |
| **Replay com a correção** | **a rodar**, esperado bytes ≤ 127 | **a rodar**, esperado `select()` 0× |
| CBMC | não lê o C++ do PX4 (`PARSING ERROR`) | idem |
| Hardware | pendente (opcional) | pendente (opcional) |

**Execução de referência no original** (PC do Bruno, 19/10/2026 08:47, ESBMC 8.5.0 + Boolector,
g++ 13.3.0): T1 FAILED, T2 SUCCESSFUL, T3 FAILED. Tempos não são de referência, porque a rodada correu
junto com outra verificação. Replay e mínimos como na tabela.

**Conferência no GitHub do PX4 (19/10/2026):** `Roboclaw.cpp`, `Roboclaw.hpp` e `module.yaml` na `main`
são idênticos aos do commit `8be2590`. No `mixer_module.cpp`, o trecho de *termination* está igual.
Busca "roboclaw" nas issues e PRs: nenhum trata destes defeitos. Decisão: não citar PRs em andamento.

---

## 3. Passo a passo

### Passo 1: reproduzir tudo, original e corrigido (💻 PC do Bruno, ~5 min)

Rode numa pasta **separada**, para não interferir em outra verificação:

```bash
mkdir -p ~/reporte && cd ~/reporte
curl -L -o saida.tar.gz https://github.com/marcela2881/dissertacao-px4-esbmc/archive/HASH_AQUI.tar.gz
tar -xzf saida.tar.gz --strip-components=2 --wildcards '*/saida/*'
[ -e px4 ] || ln -s ~/verificacoes/px4 px4
./reporte_roboclaw/rodar_reporte.sh
```

**Resultado esperado:**

| Etapa | Esperado |
|---|---|
| 1/4 ESBMC original | T1 FAILED · T2 SUCCESSFUL · T3 FAILED |
| 2/4 Replay original | 254,5–257 → 128–130 "VIOLA"; `select()` 7 vez(es); "5 situação(ões) reproduzida(s)" |
| 3/4 Mínimos | `255 → 128 … 257 → 130`; `writeCommand() returned OK (wrong)` |
| 4/4 Corrigido | ESBMC T1 e T3 **SUCCESSFUL**; replay com todos os bytes ≤ 127, `select()` 0 vez(es), "0 situação(ões) reproduzida(s)" |

Tudo fica salvo em `~/reporte/logs/<data>_bruno-pc_reporte_roboclaw/`. Se a etapa 4 não der o
esperado, **não envie o PR**: mande a saída para revisão.

### Passo 2: especificação (feito)

Manual Rev. 6.0, pp. 48–49 (`4_manual/MANUAL.md`). Só confira de passagem o texto do comando 5 na p. 49.

### Passo 3: hardware (opcional)

Com uma RoboClaw: `RBCLW_FAIL1 = 257`, acione o *flight termination* e observe o motor e, se possível,
os bytes na UART (com um analisador lógico ou um adaptador USB-serial). Sem esse teste, a issue diz
apenas que o byte sai da faixa do manual.

### Passo 4: escrever e enviar a issue

1. Leia o `5_issue/RASCUNHO_ISSUE_PT.md` para entender e reescreva o `5_issue/RASCUNHO_ISSUE_EN.md` com as suas palavras.
2. Mande a sua versão para revisão técnica.
3. Antes de enviar, procure de novo "roboclaw" nas issues do PX4 (pode ter surgido algo).
4. Envie: github.com/PX4/PX4-Autopilot → **Issues** → **New issue** → **Bug report**. Os trechos de
   código vão entre três crases (```).
5. Anote o **número da issue**: o PR precisa dele.

### Passo 5: abrir o PR com a correção (depois da issue)

1. No GitHub, faça um **fork** de PX4/PX4-Autopilot (botão *Fork*).
2. No PC do Bruno:
   ```bash
   git clone https://github.com/<seu-usuario>/PX4-Autopilot.git ~/px4-fork && cd ~/px4-fork
   git checkout -b roboclaw-fix-data-range
   git apply ~/reporte/reporte_roboclaw/6_pr/correcao_roboclaw.patch
   git diff --stat
   git commit -am "drivers/roboclaw: clamp drive data byte to 0..127 and handle write() errors"
   git push -u origin roboclaw-fix-data-range
   ```
   O `git diff --stat` deve mostrar só `src/drivers/roboclaw/Roboclaw.cpp`. Se o `git apply` falhar, o
   código mudou na `main`: avise para ajustarmos o patch.
3. No GitHub, aparece **"Compare & pull request"**. Use o seu texto reescrito a partir de
   `6_pr/RASCUNHO_PR_EN.md` e troque `#NNNNN` pelo número da issue.
4. O PX4 roda testes automáticos (CI) no PR. Se algum falhar, mande o link para análise.

---

## 4. Cuidados

- Não afirme o que acontece no motor sem ter testado no hardware.
- Deixe claro que o R1 só acontece com o **failsafe configurado acima de 254**: é uma configuração
  **permitida** pelo `module.yaml`, mas não é o padrão.
- A correção é uma **proposta**: os mantenedores podem preferir mudar o `module.yaml` ou corrigir de outro jeito.
- R3 é secundário e vai na mesma issue e no mesmo PR.
- Abra **primeiro a issue**, depois o PR citando a issue.
