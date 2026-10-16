# Configuração do módulo — lida por rodar_modulo.sh
NOME="voxl_esc"
DESCRICAO="protocolo UART do voxl_esc (qc_esc_packet.c + crc16.c)"
HARNESS="test_voxl_esc.c"
INC="src/drivers/actuators/voxl_esc"
TESTES="0 1 2 3 4"
OPCOES="--unwind 70 --overflow-check"
