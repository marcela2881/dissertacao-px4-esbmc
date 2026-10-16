# Configuração do módulo — lida por rodar_modulo.sh
NOME="roboclaw"
DESCRICAO="driver UART RoboClaw (src/drivers/roboclaw)"
HARNESS="test_roboclaw.cpp"
INC="src/drivers/roboclaw"
TESTES="0 1 2 3 4"
# harness_main: "main" fica ambíguo porque o código original chama ModuleBase::main()
OPCOES="--function harness_main --unwind 20 --overflow-check --ub-shift-check"
EXTRA_4="--nan-check"                    # opções só do teste R4
