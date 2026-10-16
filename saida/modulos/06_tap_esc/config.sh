# Configuração do módulo — lida por rodar_modulo.sh
NOME="tap_esc"
DESCRICAO="protocolo serial do tap_esc (tap_esc_common.cpp)"
HARNESS="test_tap_esc.cpp"
INC="src/drivers/tap_esc"
TESTES="0 1 2 3 4"
OPCOES="--function harness_main --unwind 130 --overflow-check"
