# GPS: reexecução com a propriedade de saída não tautológica

- `gps_prop_limpa.txt`: saída do ESBMC 8.2.0 (solver Z3 4.8.12, o padrão),
  executado pela autora em 2026-11-11 15:18 (-04) na máquina dela:
  `esbmc test_gps_prop_limpa.cpp --unwind 10 --overflow-check --no-unwinding-assertions`.
  Resultado: `VERIFICATION SUCCESSFUL`, `BMC program time: 388.744s`.
- `test_gps_prop_limpa.cpp`: `test_gps_driver.cpp` com três alterações, marcadas
  com `[reexec]` (linhas 186, 195 e 224): estado inicial válido
  (`dump_buffer.len < GPS_DUMP_DATA_SIZE`), asserção
  `dump_buffer.len <= GPS_DUMP_DATA_SIZE` e `test_choice == 3`.

## Origem destes dois arquivos

O `tee` gravou o log na máquina da autora, mas o arquivo não chegou ao
repositório. Os dois foram reconstruídos em 2026-11-12 a partir do terminal
colado por ela na conversa de trabalho: o log é o trecho do cabeçalho
"# GPS prop limpa" até "VERIFICATION SUCCESSFUL", sem alteração; o harness foi
regenerado aplicando ao `test_gps_driver.cpp` deste diretório o mesmo script de
três substituições que ela executou. As linhas `[reexec]` caem em 186, 195 e
224, como na saída do `grep` dela, e o laço citado no log
(`test_gps_prop_limpa.cpp line 66`) é o `while (len > 0)` do harness.

Uma tentativa anterior, com `--boolector`, falhou porque esse solver não está
compilado no ESBMC 8.2.0 da máquina; por isso a execução usa o Z3.
