<!--
Tradução do RASCUNHO_ISSUE_EN.md, só para você entender cada parte.
A issue é enviada em INGLÊS. Reescreva a versão em inglês com as suas palavras.
-->

# Título

Roboclaw: valores de failsafe acima de 254 geram bytes de velocidade acima de 127, e erros de write() são tratados como sucesso

## Descrição do problema

Ao verificar formalmente o driver da Roboclaw (`src/drivers/roboclaw`) com o verificador de modelos ESBMC, encontrei dois problemas. Depois confirmei os dois compilando e executando o `Roboclaw.cpp` sem modificação, com os valores dos contraexemplos.

### 1. Byte de velocidade acima de 127

O driver usa os comandos 0, 1, 4 e 5 do Packet Serial (Drive Forward/Backwards M1/M2). Segundo o manual da RoboClaw (Rev. 6.0, "Commands 0 - 7 Compatibility Commands", pp. 48–49), o byte de dados desses comandos deve estar entre 0 e 127.

O driver calcula esse byte assim:

- `updateOutputs()`: `(saída − 127) / 127`, sem limite (Roboclaw.cpp, linhas 161–162)
- `sendUnsigned7Bit()`: `lroundf(|dado| × 127)` (Roboclaw.cpp, linha 291)

Na faixa normal de saída (0 a 254), o byte fica entre 0 e 127. Porém, o `module.yaml` permite failsafe até 257. Durante o *flight termination*, o mixer envia o valor de failsafe ao driver sem limitar (mixer_module.cpp, linhas 497–501). Resultado:

| RBCLW_FAILn | byte enviado |
|---|---|
| 254 | 127 |
| 255 | **128** |
| 256 | **129** |
| 257 | **130** |

Não testei no hardware o que a RoboClaw faz com esses valores.

### 2. Erros de write() tratados como sucesso

Em `writeCommand()` e `writeCommandWithPayload()`, o retorno de `write()` é guardado num `size_t` (inteiro sem sinal). Quando `write()` falha, ele devolve −1. Num `size_t`, −1 vira um número enorme, então o teste `bytes_written < 2` dá falso e a função devolve `OK`. O driver fica esperando uma resposta que não vai chegar. No meu teste, com `write()` sempre falhando, o `readEncoder()` chamou `select()` 7 vezes.

## Como reproduzir

Problema 1, sem hardware: o programa curto faz a mesma conta das linhas 161 e 291. Saída: 254 → 127, 255 → 128, 256 → 129, 257 → 130.
No veículo: colocar `RBCLW_FAIL1` em 255, 256 ou 257 e acionar o *flight termination*.

Problema 2: fazer `write()` devolver −1 (por exemplo, substituindo o `write()` num build de teste).

## Comportamento esperado

1. O byte de velocidade nunca deveria passar de 127. Correções possíveis:
   - limitar o valor no driver (`math::constrain(...)` em `sendUnsigned7Bit()`); ou
   - mudar o limite do failsafe no `module.yaml` de 257 para 254 (igual ao `max`).
2. Um erro de `write()` deveria devolver `ERROR` (por exemplo, guardando o retorno num `ssize_t` e testando se é negativo).

## Versão do software

Commit `8be2590…` do PX4. O mesmo código continua na `main` (conferido em 19/10/2026).

## Controladora / tipo de veículo

Rover com Roboclaw. Encontrado por verificação formal e executando o código do driver no PC; sem log de voo.

## Contexto adicional

Ferramenta: ESBMC 8.5.0. A verificação inclui o `Roboclaw.cpp` original. A propriedade "byte ≤ 127" vale para saídas de 0 a 254 e falha para saídas até 257.
