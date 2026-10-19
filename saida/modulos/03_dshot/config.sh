# Configuração do módulo — lida por rodar_modulo.sh
NOME="dshot"
DESCRICAO="telemetria serial do DShot (src/drivers/dshot: DShotTelemetry, AM32Settings)"
HARNESS="test_dshot.cpp"
INC="src/drivers/dshot"
# D3 dividido por tamanho lido: testes 100 a 150 (len = TEST_CHOICE - 101, de -1 a 49)
TESTES="0 1 2 4 $(seq -s " " 100 150)"
OPCOES="--function harness_main --overflow-check --ub-shift-check"
# laços: quadro de 10 bytes e CRC8 de 9 (D0-D2); resposta de 49 bytes e CRC8 de 48 (D3-D4)
EXTRA_0="--unwind 12"
EXTRA_1="--unwind 12"
EXTRA_2="--unwind 12"
for t in $(seq 100 150); do eval "EXTRA_$t='--unwind 51'"; done
EXTRA_4="--unwind 51"
