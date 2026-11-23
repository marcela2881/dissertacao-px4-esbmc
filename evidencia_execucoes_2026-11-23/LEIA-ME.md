# Execuções de 23/11/2026 (WSL da autora)

Máquina: notebook da autora, WSL2 (Ubuntu 24.04.2, kernel 6.18.58.1), AMD Ryzen 7 5700U,
16 núcleos lógicos, 11 GiB de memória. ESBMC 8.2.0 com Z3 (sem Boolector nesta versão)
e CBMC 5.95.1. Não é a máquina da primeira campanha (i7-13700K, 64 GB, ESBMC 8.1.0,
Boolector); tempos daqui não são comparáveis com os da Tabela de resultados.

- `cbmc_dez/`: CBMC nos dez harnesses sem violação no ESBMC. O `RESUMO.txt` acumula três
  execuções do script (a primeira, com leitura como C++, falhou ao ler os arquivos, saída 6;
  a segunda deu 1 falha espúria no Navigator; a terceira, após redefinir `isfinite`, passou).
  Os logs individuais do Navigator são os da terceira.
- `cbmc_dez_antes_da_correcao_do_navigator/`: cópia dos logs antes da correção, com a falha
  espúria do Navigator (`__builtin_isfinite` sem corpo no CBMC 5.95.1).
- `pendencias_A_B_C/`: Battery com até 16 células (A), cada teste em execução separada (B) e
  GPS com k = 211 (C). O limite foi de 120 s por execução nas partes A e B.
  Atenção à parte C: os dois logs são de execuções interrompidas, sem veredito. A primeira
  foi encerrada por um monitor de memória (saída 15, 1.225 s); a segunda abortou com
  `corrupted double-linked list` (saída 134, 1.293 s) com a memória disponível abaixo de 1 GiB.
  O `RESUMO.txt` rotula as duas como "SEM VEREDITO (timeout?)", mas não foram timeout
  (o limite era de 7.200 s).
- `guarda_memoria.log`: registros do monitor de memória (encerra o GPS k = 211 se a memória
  disponível cair abaixo de 1,2 GiB).

- `inconclusivos_900s/`: os testes que ficaram sem veredito com 120 s, repetidos com limite de 900 s
  (Battery 16 células, EKF2 teste 1, GPS testes 0 a 3, VTOL testes 0 e 1; o teste 4 do GPS não foi
  repetido). Os logs de cada teste estão nesta pasta; o `RESUMO.txt` traz os vereditos.
- `pendencias_A_B_C/C_gps_k211_original_cortado_pela_reinicializacao_do_windows.log`: primeira
  tentativa do GPS com k = 211, cortada por uma reinicialização do Windows; termina na mesma linha
  do desenrolamento (iteração 211 do laço do `memcpy`), sem veredito e sem linha de saída.

- `cbmc_longo_7200s/`: CBMC 5.95.1 repetido no Battery e no EKF2, com e sem asserções de
  desenrolamento, limite de 7.200 s. As quatro execuções terminam sem veredito (saída 124). O
  `RESUMO.txt` registra as quatro; o log individual do Battery sem asserções foi regravado por
  uma segunda execução do script (iniciada por engano) e, por isso, é só o cabeçalho de uma
  rodada interrompida, sem linha de saída. O resultado dele é o do `RESUMO.txt`. O solver
  (MiniSAT 2.2.1) recebeu 36.030 variáveis e 154.528 cláusulas no Battery e 40.904 variáveis e
  178.083 cláusulas no EKF2.
- `roboclaw_uninit/`: reprodução do padrão de memória não inicializada do RoboClaw (Valgrind 3.22.0
  e MSan com clang 18.1.3), executada pela autora em 23/11/2026 (05:07 UTC). Valgrind no original:
  saída 9 e três avisos de valor não inicializado; no corrigido: saída 0 e nenhum aviso. Com
  memória suja, o original ficou bloqueado até o limite de 15 s; o corrigido terminou. O MSan não
  detectou o problema (0 ocorrências) e não é usado como evidência. Os executáveis não foram
  arquivados.
