/**
 * @file replay.cpp — candidato 8 (MicroStrain.cpp)
 *
 * REPLAY do contraexemplo do ESBMC/CBMC para o candidato 8 (MicroStrain.cpp).
 *
 * MODULO PX4: src/drivers/ins/microstrain/MicroStrain.cpp
 * FUNCAO:     MicroStrain::MicroStrain() — construtor, linhas 53-57
 *
 * ---------------------------------------------------------------------------
 * POR QUE ESTA VERSAO SUBSTITUI O replay_microstrain.cpp
 * ---------------------------------------------------------------------------
 * O replay anterior fixava o cenario critico por atribuicao direta:
 *
 *     size_t max_len = PORT_SIZE;      // 128, atribuido, nao calculado
 *     _port[max_len] = '\0';
 *
 * Isso demonstra apenas que _port[128] estoura um array de 128 posicoes — o
 * que e aritmetica, nao replay. Um replay existe para mostrar que o CAMINHO
 * REAL do driver conduz ao estado violado a partir de uma entrada concreta.
 *
 * Este arquivo replica as linhas 53-57: strnlen e math::min sao
 * calculados, e max_len sai deles. A entrada e o contraexemplo exato que o
 * ESBMC construiu no harness test_microstrain_port_v3.cpp:
 *
 *     uart_port[0..127] = bytes nao-nulos
 *     uart_port[128]    = '\0'
 *     -> strnlen(uart_port, 128) = 128
 *     -> max_len = min(128, 128) = 128
 *     -> _port[128] = '\0'  <-- fora dos limites de _port[128]
 *
 * ---------------------------------------------------------------------------
 * CODIGO REAL REPLICADO (MicroStrain.cpp, construtor)
 * ---------------------------------------------------------------------------
 *     const size_t max_len = math::min(sizeof(_port), strnlen(uart_port, sizeof(_port)));
 *     strncpy(_port, uart_port, max_len);
 *     _port[max_len] = '\0';
 *
 * ---------------------------------------------------------------------------
 * COMPILACAO
 * ---------------------------------------------------------------------------
 *   g++ -fsanitize=address -g -O0 -o r8 replay_microstrain_v3.cpp && ./r8
 *
 * O _port e alocado sozinho no heap para que o AddressSanitizer veja a
 * escrita. No PX4 real _port e membro do objeto MicroStrain (criado com new
 * em MicroStrain::task_spawn); la o byte extra cai no membro seguinte do
 * objeto (os parametros declarados depois de _port no .hpp), dentro da mesma
 * alocacao, onde o ASan nao o veria. O replay mostra o indice fora do
 * array; nao mostra o efeito no objeto real.
 *
 * v5 (C7): sem free() depois da escrita fora dos limites.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define PORT_SIZE 128

/* Replica math::min do PX4. */
static size_t min_px4(size_t a, size_t b)
{
    return (a < b) ? a : b;
}

int main(void)
{
    printf("=== Replay do candidato 8: MicroStrain.cpp linha 55 ===\n");
    printf("Contraexemplo ESBMC: uart_port com %d bytes nao-nulos\n", PORT_SIZE);
    printf("Esperado: heap-buffer-overflow na escrita _port[max_len]\n\n");

    /* _port do driver: array de PORT_SIZE, indices validos 0..PORT_SIZE-1 */
    char *_port = (char *)malloc(PORT_SIZE);
    memset(_port, 0, PORT_SIZE);

    /* ENTRADA — contraexemplo exato produzido pelo ESBMC:
     * PORT_SIZE bytes nao-nulos seguidos de terminador na posicao PORT_SIZE. */
    char uart_port[PORT_SIZE + 1];
    memset(uart_port, 'A', PORT_SIZE);
    uart_port[PORT_SIZE] = '\0';

    /* LINHA 53 DO MicroStrain.cpp — replicada com os calculos reais.
     * max_len NAO e atribuido: sai de strnlen + min, como no driver. */
    const size_t max_len = min_px4(sizeof(char[PORT_SIZE]),
                                   strnlen(uart_port, PORT_SIZE));

    printf("strnlen(uart_port, %d) = %zu\n", PORT_SIZE, strnlen(uart_port, PORT_SIZE));
    printf("max_len = min(%d, %zu) = %zu\n", PORT_SIZE, strnlen(uart_port, PORT_SIZE), max_len);
    printf("indices validos de _port: 0..%d\n\n", PORT_SIZE - 1);

    /* Copia de max_len bytes — cabe em _port[0..127], e legitima. */
    strncpy(_port, uart_port, max_len);
    printf("strncpy de %zu bytes: OK (cabe no buffer)\n", max_len);

    printf("Escrevendo _port[%zu] = '\\0' ...\n", max_len);

    /* A ESCRITA PROBLEMATICA: _port[128] num array de 128 posicoes. */
    _port[max_len] = '\0';

    printf("Se chegou aqui: bug nao detectado (UB silencioso)\n");
    return 0;
}
