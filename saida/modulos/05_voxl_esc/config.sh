# Configuração do módulo — lida por rodar_modulo.sh
NOME="voxl_esc"
DESCRICAO="protocolo UART do voxl_esc (qc_esc_packet.c + crc16.c)"
HARNESS="test_voxl_esc.c"
INC="src/drivers/actuators/voxl_esc"
TESTES="0 1 2 3 4 5 6 7 8"   # T1, T5-T8: V1 por tamanho de payload (0-1, 2, 4, 6, 10)
OPCOES="--unwind 70 --overflow-check"
