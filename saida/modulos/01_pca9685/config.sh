# Configuração do módulo — lida por rodar_modulo.sh
NOME="pca9685"
DESCRICAO="driver PWM I2C PCA9685 (src/drivers/pca9685_pwm_out)"
HARNESS="test_pca9685.cpp"
INC="src/drivers/pca9685_pwm_out"        # pastas do PX4 para -I (relativas ao PX4)
TESTES="0 1 2 3 4"
OPCOES="--unwind 70 --overflow-check"    # iguais às da rodada de referência de 15/10/2026
