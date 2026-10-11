// Item 4.2: com o buffer real de 79 bytes, a violacao e alcancavel?
// Prova por inducao do invariante I: dump_data->len < sizeof(dump_data->data).
//  Base:  GPS::initializeCommunicationDump() faz memset(..., 0) nos dois buffers
//         (gps.cpp linhas 717-718) => len = 0 => I vale na primeira chamada.
//  Passo: este harness. Parte de um estado ARBITRARIO que satisfaz I (len_d
//         nao determinístico em [0,78]), com len de entrada arbitrario > 0,
//         executa UMA iteracao do laco e verifica (a) memcpy dentro dos limites
//         (checagem automatica de ponteiro/limites) e (b) I continua valendo.
// Sem desenrolamento: o resultado vale para qualquer numero de iteracoes e chamadas
// (desde que dumpGpsData seja o unico escritor de len -- conferir no gps.cpp).
#include "prod_common.h"
int main() {
    struct gps_dump_s dump;
    dump.len = nondet_uint8_t();
    ASSUME(dump.len < sizeof(dump.data));          // hipotese de inducao I

    size_t len = nondet_size_t();
    ASSUME(len > 0 && len <= 4096);                  // origem: buffer do chamador
    static uint8_t src[4096];                                  // buffer do chamador, >= len
    const uint8_t *data = src;

    loop_body(&data, &len, &dump, nondet_bool());

    assert(dump.len < sizeof(dump.data));                      // I preservado
    return 0;
}
