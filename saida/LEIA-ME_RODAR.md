# Como rodar as verificações (passo a passo)

🖥️ = rodar na **fm07** · 💻 = rodar no **PC (WSL)**

Copie sempre **só o que está dentro das caixas de comando**. O que aparece fora
delas é explicação ou exemplo de saída, não comando.

---

## 1. Baixar ou atualizar os arquivos (🖥️ fm07)

Use isto na primeira vez e sempre que eu avisar que há módulos novos. Os logs e
o PX4 já baixado **não são apagados**.

```bash
mkdir -p ~/verificacoes && cd ~/verificacoes
curl -sL https://github.com/marcela2881/dissertacao-px4-esbmc/archive/refs/heads/claude/brave-euler-o09lja.tar.gz | tar -xz --strip-components=2 --wildcards '*/saida/*'
ls modulos
```

O `ls` mostra os módulos disponíveis (ex.: `01_pca9685  02_roboclaw  05_voxl_esc`).

## 2. Teste rápido de um módulo (🖥️ fm07)

Serve para descobrir **problemas da ferramenta** (travamento, erro de conversão)
antes da rodada oficial. Uma execução por teste, timeout de 5 min por teste.

```bash
cd ~/verificacoes
./rodar_modulo.sh roboclaw rapido
```

Na primeira vez, o script baixa o PX4 no commit fixado (alguns minutos).
No fim aparece um bloco assim (exemplo de saída):

```
===== RESUMO: roboclaw (rapido) =====
maquina: fm07 | ESBMC 8.5.0 | solver boolector | 16/10/2026 10:00
roboclaw_T0_ua: mediana = 1.234s |  1 [VERIFICATION SUCCESSFUL]
...
```

➡️ **Cole esse bloco RESUMO na conversa.** Eu digo se está pronto para a rodada
oficial ou se o harness precisa de ajuste.

## 3. Rodada oficial de um módulo (🖥️ fm07)

Só depois que o teste rápido estiver ok. Três execuções por teste (mediana),
com e sem asserções de desdobramento. Pode demorar, então rode em segundo plano
(continua mesmo se a conexão cair):

```bash
cd ~/verificacoes
nohup ./rodar_modulo.sh roboclaw oficial > oficial_roboclaw.txt 2>&1 &
```

Para ver se terminou (aparece o RESUMO no fim):

```bash
cd ~/verificacoes
tail -15 oficial_roboclaw.txt
```

➡️ **Cole o RESUMO na conversa.** Se houver `FAILED`, eu aviso se é candidato a
reporte e passo o checklist para a sua análise.

Para rodar **todos** os módulos de uma vez (rápido ou oficial):

```bash
cd ~/verificacoes
nohup ./rodar_tudo.sh oficial > oficial_tudo.txt 2>&1 &
```

## 4. Levar os logs para o PC (no final)

🖥️ fm07:

```bash
cd ~/verificacoes
./scripts/empacotar_logs.sh
```

💻 PC (WSL):

```bash
mkdir -p ~/pesquisa-px4 && scp p79642ma@fm07.cs.manchester.ac.uk:~/logs_saida_*.tar.gz ~/pesquisa-px4/
```

Subir para o git só no final, quando tudo estiver pronto (combinado).

---

## Módulos

| Pasta | Módulo | Testes | Situação |
|---|---|---|---|
| `01_pca9685` | driver PWM I2C PCA9685 | T0–T4 | rodada oficial feita (15/10/2026) |
| `02_roboclaw` | driver UART RoboClaw | T0–T4 (R0–R4) | falta teste rápido |
| `03_dshot` | DShot | — | em preparação |
| `04_uavcan_actuators` | UAVCAN ESC/servo | — | em preparação |
| `05_voxl_esc` | protocolo UART voxl_esc | T0–T8 (V0–V4; V1 por tamanho) | teste rápido em andamento |
| `06_tap_esc` | protocolo serial TAP ESC | T0–T4 | falta teste rápido |
| `07_px4io` | PX4IO | — | em preparação |
| `08_mixer_module` | mixer_module | — | em preparação |
| `09_control_allocation` | biblioteca control_allocation | — | em preparação |
| `10_control_allocator` | módulo control_allocator | — | em preparação |

## Se algo der errado

- `ERRO: ESBMC não encontrado`: rode `export ESBMC=$(find ~/esbmc-8.5 -name esbmc -type f | head -1)` e repita.
- `TRAVOU`, `CONVERSION ERROR` ou `PARSING ERROR` no RESUMO: é problema de
  ferramenta ou de harness, não resultado. Cole o RESUMO na conversa.
- `Timed out`: o teste não terminou no tempo limite. Também é um resultado; cole na conversa.
