# Configuração do módulo — lida por rodar_modulo.sh e comparar.sh
NOME="mixer_module"
DESCRICAO="mixer de saída (src/lib/mixer_module, MixingOutput: limites, failsafe, rampa)"
HARNESS="test_mixer_module.cpp"
# mixer_module: o .cpp original; lib: mathlib e matrix originais. src/ NÃO entra, para que
# drivers/drv_pwm_output.h e o uORB venham dos stubs.
INC="src/lib/mixer_module src/lib"
TESTES="0 1 2 3 4 5"
# laços: 16 canais (MAX_ACTUATORS) nos laços de inicialização e destruição
OPCOES="--function harness_main --unwind 20 --overflow-check"
