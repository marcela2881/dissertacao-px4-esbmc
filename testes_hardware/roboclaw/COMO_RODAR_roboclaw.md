# Verificação formal do RoboClaw (para entrar na contagem)

Até agora o RoboClaw tinha só análise de código + teste com UART emulada.
Este harness faz a verificação formal (ESBMC/CBMC) do **Bug 2**, com o mesmo
critério dos outros módulos. Rode na sua máquina para gerar o log oficial.

## O bug (em uma frase)
No `writeCommandWithPayload()`, o retorno de `write()` é guardado num `size_t`.
Quando `write()` falha e devolve -1, o valor vira um número gigante, a checagem
não percebe e a função retorna OK mesmo tendo falhado.

## Comandos

```
cd ~/Downloads      # ou a pasta onde você salvar o arquivo

# ESBMC — sem correção (espera VERIFICATION FAILED)
esbmc harness_roboclaw_bug2.cpp --overflow-check

# ESBMC — com correção (espera VERIFICATION SUCCESSFUL)
esbmc harness_roboclaw_bug2.cpp -DCORRECAO --overflow-check

# CBMC — sem correção (espera VERIFICATION FAILED)
cbmc harness_roboclaw_bug2.cpp --signed-overflow-check

# CBMC — com correção (espera VERIFICATION SUCCESSFUL)
cbmc harness_roboclaw_bug2.cpp -DCORRECAO --signed-overflow-check
```

Não precisa de `--unwind` (não há laços).

## Resultado esperado
- Sem correção: **VERIFICATION FAILED** (o contraexemplo é `write() = -1`).
- Com correção (`-DCORRECAO`): **VERIFICATION SUCCESSFUL**.

Já validei aqui com o CBMC 5.95.1: FAILED sem correção, SUCCESSFUL com a
correção. Faltam os logs da sua máquina (ESBMC + CBMC) para oficializar.

## Para guardar a evidência (como no batmon)
```
mkdir -p ~/evidencia_roboclaw && cd ~/Downloads
{ echo "# RoboClaw Bug 2 — maquina da Marcela"; date; esbmc --version | head -1; cbmc --version; echo;
  echo "== ESBMC sem correcao =="; esbmc harness_roboclaw_bug2.cpp --overflow-check 2>&1 | tail -3;
  echo "== ESBMC com correcao =="; esbmc harness_roboclaw_bug2.cpp -DCORRECAO --overflow-check 2>&1 | tail -3;
  echo "== CBMC sem correcao ==";  cbmc  harness_roboclaw_bug2.cpp --signed-overflow-check 2>&1 | tail -3;
  echo "== CBMC com correcao ==";  cbmc  harness_roboclaw_bug2.cpp -DCORRECAO --signed-overflow-check 2>&1 | tail -3;
} | tee ~/evidencia_roboclaw/roboclaw_bug2_maquina.txt
cp harness_roboclaw_bug2.cpp ~/evidencia_roboclaw/
cd ~/evidencia_roboclaw && sha256sum * > SHA256SUMS.txt && cat SHA256SUMS.txt
```

## Observação
- **Bug 1** (byte de velocidade passa de 127 na terminação de voo) também é
  verificável, mas depende de modelar como o `updateOutputs()` mapeia o valor de
  failsafe — fazemos num segundo passo, se você quiser.
- Nada vai ao PX4 sem a revisão do Prof. Lucas.
