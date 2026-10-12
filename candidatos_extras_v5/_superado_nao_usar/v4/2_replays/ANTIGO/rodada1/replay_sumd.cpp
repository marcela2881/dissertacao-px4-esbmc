#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define SUMD_MAX_CHANNELS 32
#define SUMD_HEADER_ID    0xA8
#define SUMD_ID_SUMD      0x01

struct ReceiverFcPacketHoTT {
    uint8_t header;
    uint8_t status;
    uint8_t length;
    uint8_t sumd_data[SUMD_MAX_CHANNELS * 2]; // [64], índices 0..63
    uint8_t crc16_high;
    uint8_t crc16_low;
    uint8_t telemetry;
    uint8_t crc8;
};

int main() {
    ReceiverFcPacketHoTT *pkt = (ReceiverFcPacketHoTT*)malloc(sizeof(ReceiverFcPacketHoTT));
    memset(pkt, 0, sizeof(ReceiverFcPacketHoTT));

    pkt->header = SUMD_HEADER_ID;
    pkt->status = SUMD_ID_SUMD;
    pkt->length = SUMD_MAX_CHANNELS; // 32 — valor máximo LEGÍTIMO

    uint8_t rxlen = 1; // começa em 1 (após processar length)

    printf("length=%d sumd_data tem %d posicoes (indices 0..%d)\n",
           pkt->length, SUMD_MAX_CHANNELS*2, SUMD_MAX_CHANNELS*2 - 1);
    printf("Loop escreve ate rxlen <= length*2 = %d\n", pkt->length * 2);

    // Loop real de sumd.cpp — condicao <= deveria ser <
    while (rxlen <= (pkt->length * 2)) {
        pkt->sumd_data[rxlen] = 0xAA; // escrita — em rxlen=64 vai alem do array
        if (rxlen == SUMD_MAX_CHANNELS * 2) {
            printf("Tentando escrever em sumd_data[%d] (ultimo invalido)...\n", rxlen);
        }
        rxlen++;
    }

    printf("rxlen final=%d\n", rxlen);
    free(pkt);
    return 0;
}
