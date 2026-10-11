// Controle negativo do item 4.2: MESMO corpo de producao, mas len_d irrestrito
// (0..255), isto e, sem a hipotese I. Esperado: violacao de limites no memcpy
// quando len_d > 79 (sizeof(data) - len_d em size_t da a volta). Mostra que o
// harness indutivo detectaria o defeito se I nao valesse.
#include "prod_common.h"
int main() {
    struct gps_dump_s dump;
    dump.len = nondet_uint8_t();                               // SEM hipotese I

    size_t len = nondet_size_t();
    __CPROVER_assume(len > 0 && len <= 4096);
    static uint8_t src[4096];                                  // buffer do chamador, >= len
    const uint8_t *data = src;

    loop_body(&data, &len, &dump, nondet_bool());
    return 0;
}
