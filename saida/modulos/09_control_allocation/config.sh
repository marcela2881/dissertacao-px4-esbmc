# Configuração do módulo — lida por rodar_modulo.sh e comparar.sh
NOME="control_allocation"
DESCRICAO="alocação de controle (src/lib/control_allocation: clip, slew rate, normalização, dessaturação)"
HARNESS="test_control_allocation.cpp"
# control_allocation: os .cpp originais; lib: mathlib e matrix originais (src/ não entra)
INC="src/lib/control_allocation src/lib"
TESTES="0 1 2 3"
# laços: 16 atuadores (NUM_ACTUATORS) e 6 eixos; geninv percorre a matriz 6x16
OPCOES="--function harness_main --unwind 17 --overflow-check"
