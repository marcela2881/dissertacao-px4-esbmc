/**
 * @file test_microstrain_port_v3.cpp
 *
 * MODULO: src/drivers/ins/microstrain/MicroStrain.cpp
 * FUNCAO: MicroStrain::MicroStrain() — construtor, linha 55
 * METODO: Bounded Model Checking com ESBMC / CBMC
 *
 * ---------------------------------------------------------------------------
 * POR QUE ESTA VERSAO EXISTE (v3 substitui a v2)
 * ---------------------------------------------------------------------------
 * O harness v2 fixava o cenario critico por constante:
 *
 *     const size_t max_len = PORT_SIZE;   // 128, atribuido diretamente
 *     _port[max_len] = '\0';
 *
 * Com isso, o verificador demonstrava apenas que _port[128] estoura um array
 * de 128 posicoes — o que e aritmetica, nao verificacao. A pergunta que
 * interessa e outra: EXISTE ENTRADA que leve o driver real a max_len = 128?
 * O harness v2 assumia a resposta em vez de deriva-la.
 *
 * A v2 tambem trazia, no main(), a assercao
 *
 *     assert(PORT_SIZE <= PORT_SIZE - 1);   // 128 <= 127
 *
 * que e uma contradicao — falsa para qualquer execucao. Ela nunca chegou a ser
 * alcancada (o ESBMC para na primeira violacao, que ocorre antes, na escrita),
 * mas era ruido e foi removida.
 *
 * ---------------------------------------------------------------------------
 * O QUE A v3 FAZ DIFERENTE
 * ---------------------------------------------------------------------------
 * 1. uart_port passa a ser NAO-DETERMINISTICO: cada byte e livre, inclusive a
 *    possibilidade de nao haver terminador nul nos primeiros PORT_SIZE bytes.
 * 2. strnlen e calculado, nao assumido — replicando a semantica da libc.
 * 3. max_len sai do min(sizeof(_port), strnlen(...)), como no driver real.
 * 4. A propriedade verificada e o limite de _port na escrita, que o ESBMC ja
 *    checa automaticamente, mais uma assercao explicita que exprime a mesma
 *    condicao de forma legivel no log.
 *
 * Assim o verificador decide se o estado critico e alcancavel a partir da
 * entrada, em vez de parti-lo como premissa.
 *
 * CODIGO REAL REPLICADO (MicroStrain.cpp, construtor):
 *     const size_t max_len = math::min(sizeof(_port),
 *                                      strnlen(uart_port, sizeof(_port)));
 *     memcpy(_port, uart_port, max_len);
 *     _port[max_len] = '\0';
 */

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define PORT_SIZE 128

extern char nondet_char(void);

static char _port[PORT_SIZE] = {};

/* Buffer de entrada com uma posicao a mais que PORT_SIZE, para admitir o caso
 * em que os PORT_SIZE primeiros bytes sao todos nao-nulos. */
static char uart_port[PORT_SIZE + 1];

/* Replica strnlen(s, maxlen) da libc: percorre ate encontrar '\0' ou atingir
 * maxlen, o que vier primeiro. Retorna o comprimento encontrado. */
static size_t strnlen_model(const char *s, size_t maxlen)
{
    size_t i = 0;
    while (i < maxlen && s[i] != '\0') {
        i++;
    }
    return i;
}

/* Replica math::min do PX4. */
static size_t min_model(size_t a, size_t b)
{
    return (a < b) ? a : b;
}

void microstrain_init_port(void)
{
    /* ENTRADA LIVRE: cada byte de uart_port e nao-deterministico.
     * Nenhuma suposicao sobre haver ou nao terminador nul. */
    for (size_t i = 0; i < PORT_SIZE + 1; i++) {
        uart_port[i] = nondet_char();
    }

    /* Linha 55 do MicroStrain.cpp, replicada: */
    const size_t max_len = min_model(sizeof(_port),
                                     strnlen_model(uart_port, sizeof(_port)));

    memcpy(_port, uart_port, max_len);

    /* PROPRIEDADE: o indice escrito precisa ser um indice valido de _port,
     * isto e, estritamente menor que PORT_SIZE. Se o solver encontrar entrada
     * que leve max_len a PORT_SIZE, esta assercao e violada — e a escrita
     * abaixo estoura o array. */
    assert(max_len < PORT_SIZE);

    _port[max_len] = '\0';
}

int main()
{
    microstrain_init_port();
    return 0;
}
