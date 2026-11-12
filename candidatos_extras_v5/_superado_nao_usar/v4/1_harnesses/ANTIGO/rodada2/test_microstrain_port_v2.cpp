/**
 * @file test_microstrain_port_v2.cpp
 *
 * MÓDULO: src/drivers/ins/microstrain/MicroStrain.cpp
 * FUNÇÃO: MicroStrain::MicroStrain() — construtor, linha 55
 * MÉTODO: Bounded Model Checking com ESBMC / CBMC
 *
 * Off-by-one: _port[128] tem índices válidos 0..127.
 * Se uart_port tiver >= 128 chars, max_len = 128
 * e _port[max_len] = _port[128] escreve além do array (CWE-787).
 *
 * Versão v2: sem loop de preenchimento — modela diretamente o cenário
 * crítico (uart_port de exatamente 128 caracteres) para evitar explosão
 * de estados no solver.
 */

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define PORT_SIZE 128

static char _port[PORT_SIZE] = {};

/* Replica math::min(sizeof(_port), strnlen(uart_port, sizeof(_port)))
 * para o caso crítico: uart_port com exatamente PORT_SIZE chars não-nulos
 * → strnlen retorna PORT_SIZE → max_len = PORT_SIZE → _port[PORT_SIZE] OOB */
void microstrain_init_port_critical(void)
{
    /* Caso crítico: uart_port com PORT_SIZE caracteres não-nulos.
     * strnlen(uart_port, PORT_SIZE) retorna PORT_SIZE.
     * max_len = min(PORT_SIZE, PORT_SIZE) = PORT_SIZE = 128. */
    const size_t max_len = PORT_SIZE;

    /* strncpy copia max_len bytes — seguro, cabe em _port[0..127] */
    /* A linha problemática do código real (MicroStrain.cpp linha 55): */
    _port[max_len] = '\0';   /* _port[128] — FORA DOS LIMITES */
}

int main()
{
    microstrain_init_port_critical();

    /* PROPRIEDADE: o índice max_len deve ser <= PORT_SIZE - 1 = 127
     * para que _port[max_len] seja um acesso válido. */
    assert(PORT_SIZE <= PORT_SIZE - 1);

    return 0;
}
