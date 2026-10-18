# Configuração do módulo — lida por rodar_modulo.sh
NOME="tap_esc"
DESCRICAO="protocolo serial do tap_esc (tap_esc_common.cpp)"
HARNESS="test_tap_esc.cpp"
INC="src/drivers/tap_esc"
TESTES="0 1 2 3 4"
# --unwind por teste: T1 e T3 percorrem até 128/101 bytes; T0, T2 e T4 têm laços curtos
# (um --unwind global de 130 fazia o ESBMC desenrolar o CRC à toa -> Timed out)
OPCOES="--function harness_main --overflow-check"
EXTRA_0="--unwind 20"
EXTRA_1="--unwind 130"
EXTRA_2="--unwind 20"
EXTRA_3="--unwind 130"
EXTRA_4="--unwind 20"
