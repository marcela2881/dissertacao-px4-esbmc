# RoboClaw — evidência da verificação formal (máquina da Marcela)

Verificação formal dos dois defeitos do driver `src/drivers/roboclaw/Roboclaw.cpp`
(PX4 commit `8be2590c1529517ccd54fbc09ae138402517baf4`), executada na máquina da
Marcela em 28/10/2026.

## Ferramentas
- **ESBMC 8.2.0** (64-bit, Linux; solver Z3)
- **CBMC 5.95.1**

As versões ficam registradas no topo de `roboclaw_maquina.txt`
(`ESBMC version 8.2.0 64-bit x86_64 linux` e `5.95.1 (cbmc-5.95.1)`).
A campanha dos módulos do artigo usou ESBMC 8.1.0 (Boolector).

## Resultado (ver `roboclaw_maquina.txt`)

| Defeito | sem correção | com `-DCORRECAO` |
|---|---|---|
| **Bug 1** — byte de velocidade > 127 (`sendUnsigned7Bit`) | VERIFICATION FAILED | VERIFICATION SUCCESSFUL |
| **Bug 2** — erro de `write()` não detectado (`writeCommandWithPayload`) | VERIFICATION FAILED | VERIFICATION SUCCESSFUL |

Mesmo padrão nas duas ferramentas (ESBMC e CBMC): o verificador encontra o
contraexemplo no código original e prova a ausência da violação com a correção
do `correcao_roboclaw.patch`.

## Integridade
`SHA256SUMS.txt` cobre `roboclaw_maquina.txt` e os dois harness. Para conferir:

```
cd testes_hardware/roboclaw/evidencia_maquina
sha256sum -c SHA256SUMS.txt
```

Os harness são os mesmos de `testes_hardware/roboclaw/` (`harness_roboclaw_bug1.cpp`,
`harness_roboclaw_bug2.cpp`).

## Observação
Nada foi reportado ao PX4. Qualquer report à comunidade depende da revisão do
Prof. Lucas.
