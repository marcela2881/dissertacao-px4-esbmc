# Como replicar o achado do batmon

Arquivos: `harness.cpp` (verificação formal) e `replay.cpp` (execução concreta).
Base: PX4 commit `8be2590c15` (conferido também no `main` em 13/10/2026).

## O defeito, em uma frase
O nº de células vem da bateria (SMBus, registrador `0x40`) e é limitado a
`MAX_CELL_COUNT = 16`, mas `battery_status_s::voltage_cell_v` tem só 14
posições. Com 15 ou 16 células, o driver escreve fora do vetor.

## 1. Execução concreta com sanitizers (UBSan/ASan)
Precisa só de `g++`.

```
g++ -g -fsanitize=address,undefined replay.cpp -o replay_batmon
./replay_batmon 14    # sem erro; full_charge_capacity_wh = 0
./replay_batmon 15    # UBSan: index 14 out of bounds for type 'float [14]'
./replay_batmon 16    # idem; full_charge_capacity_wh = 3.7150 (tensao da 16a celula)
```

## 2. Verificação formal com ESBMC
```
esbmc harness.cpp --unwind 20 --overflow-check                 # VERIFICATION FAILED
esbmc harness.cpp -DCORRECAO --unwind 20 --overflow-check      # VERIFICATION SUCCESSFUL
```

## 3. Verificação com CBMC
```
cbmc harness.cpp --unwind 20 --unwinding-assertions --bounds-check --pointer-check              # 1 de N falha -> FAILED
cbmc harness.cpp -DCORRECAO --unwind 20 --unwinding-assertions --bounds-check --pointer-check   # 0 de N -> SUCCESSFUL
```

`-DCORRECAO` aplica a correção proposta: limitar `_cell_count` também ao
tamanho de `voltage_cell_v`, como o PR #28606 fez no driver UAVCAN.

## Resultados obtidos
- `resultados_maquina/` — execução na máquina da Marcela (WSL Ubuntu),
  ESBMC 8.2.0, CBMC 5.95.1, UBSan. Com `SHA256SUMS.txt`.
- `resultados_nuvem/` — execução anterior na nuvem, ESBMC 8.1.0 e 8.5.0,
  CBMC 5.95.1, UBSan.

Em todos os ambientes: sem correção FALHA, com correção PASSA.

## Situação
- Achado **ainda não reportado** ao PX4.
- Mesmo bug corrigido no UAVCAN (PR #28606); o batmon ficou de fora.
- Rascunho da issue: `ISSUE_batmon.md`.
- **Não publicar sem a revisão do Prof. Lucas.**
