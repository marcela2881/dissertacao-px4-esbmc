# Configuração do módulo — lida por rodar_modulo.sh
NOME="dshot"
DESCRICAO="telemetria serial do DShot (src/drivers/dshot: DShotTelemetry, AM32Settings)"
HARNESS="test_dshot.cpp"
INC="src/drivers/dshot"
TESTES="0 1 2 3 4"
OPCOES="--function harness_main --overflow-check --ub-shift-check"
# laços: quadro de 10 bytes e CRC8 de 9 (D0-D2); resposta de 49 bytes e CRC8 de 48 (D3-D4)
EXTRA_0="--unwind 12"
EXTRA_1="--unwind 12"
EXTRA_2="--unwind 12"
EXTRA_3="--unwind 51"
EXTRA_4="--unwind 51"
