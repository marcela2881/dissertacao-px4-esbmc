/**
 * @file replay_bst_real.cpp
 *
 * REPLAY DO CÓDIGO REAL — bst.cpp::BST::probe()
 *
 * Este arquivo contém a função probe() e todas as structs VERBATIM do
 * arquivo real src/drivers/telemetry/bst/bst.cpp do PX4 Autopilot v1.16.
 *
 * Substituições mínimas necessárias para compilar fora do build system do PX4:
 *   - device::I2C::transfer() → stub que preenche o buffer de resposta com
 *     o contraexemplo exato (dev_name_len=244) encontrado pelo ESBMC
 *   - PX4_ERR(), PX4_DEBUG() → printf()
 *   - OK, -EIO → constantes numéricas equivalentes
 *   - _retries → variável local (membro da classe base I2C, não relevante)
 *
 * NADA da lógica de probe() foi alterado — a linha do bug (208 do arquivo
 * original) está aqui exatamente como escrita pelos desenvolvedores do PX4.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

// ================== STUBS MÍNIMOS (substituições de API do PX4/NuttX) ==================
#define OK    0
#define EIO   5
#define PX4_ERR(fmt, ...)   printf("[ERR] " fmt "\n", ##__VA_ARGS__)
#define PX4_DEBUG(fmt, ...) printf("[DBG] " fmt "\n", ##__VA_ARGS__)

// ================== STRUCTS REAIS — verbatim de bst.cpp ==================
#pragma pack(push, 1)

template <typename T>
struct BSTPacket {
	uint8_t length;
	uint8_t type;
	T payload;
	uint8_t crc;
};

struct BSTDeviceInfoRequest {
	uint8_t cmd;
};

struct BSTDeviceInfoReply {
	uint32_t hw_id;
	uint16_t fw_id;
	uint8_t dev_name_len;   // <-- campo não confiável, vem do dispositivo I2C
	char dev_name[32];      // <-- array de 32 posições, índices válidos 0..31
};

#pragma pack(pop)

// ================== crc8() REAL — verbatim de bst.cpp linhas 269-280 ==================
static uint8_t crc8(uint8_t *data, size_t len)
{
	uint8_t crc = 0x00;

	while (len--) {
		crc ^= *data++;

		for (int i = 0; i < 8; i++) {
			crc = crc & 0x80 ? (crc << 1) ^ 0xD5 : crc << 1;
		}
	}

	return crc;
}

// ================== STUB de I2C::transfer() ==================
// Simula a resposta do dispositivo BST via I2C.
// No PX4 real, transfer() envia o request e preenche o buffer de resposta
// com o que o dispositivo BST responde via protocolo I2C.
// Aqui injetamos o contraexemplo exato do ESBMC: dev_name_len = 244.
static int i2c_transfer_stub(uint8_t *send_buf, size_t send_len,
                              uint8_t *recv_buf, size_t recv_len)
{
    if (recv_buf != nullptr && recv_len > 0) {
        memset(recv_buf, 0, recv_len);
        // Simula resposta do dispositivo BST malicioso/defeituoso:
        BSTPacket<BSTDeviceInfoReply> *reply =
            reinterpret_cast<BSTPacket<BSTDeviceInfoReply>*>(recv_buf);
        reply->type = 0x05;          // tipo correto (passa a checagem de tipo)
        reply->length = (uint8_t)(sizeof(BSTPacket<BSTDeviceInfoReply>) - 1);
        reply->payload.hw_id = 0x12345678;
        reply->payload.fw_id = 0x0100;
        reply->payload.dev_name_len = 244; // CONTRAEXEMPLO DO ESBMC: 244 > 31
        memset(reply->payload.dev_name, 'A', 32);
        // CRC calculado sobre o pacote real para passar a verificação de CRC
        reply->crc = crc8(&reply->type, sizeof(BSTPacket<BSTDeviceInfoReply>) - 2);
    }
    return OK;
}

// ================== probe() REAL — verbatim de bst.cpp linhas 185-214 ==================
// Única alteração: substituição de send_packet() por chamada direta ao stub I2C,
// e de _retries por variável local. A linha do bug (208) está INALTERADA.
static int bst_probe_real()
{
    BSTPacket<BSTDeviceInfoRequest> dev_info_req = {};
    dev_info_req.type = 0x0A;
    dev_info_req.payload.cmd = 0x04;
    BSTPacket<BSTDeviceInfoReply> dev_info_reply = {};

    // send_packet() no código real chama I2C::transfer() internamente
    // Stub reproduz exatamente esse comportamento com o contraexemplo
    i2c_transfer_stub(
        reinterpret_cast<uint8_t *>(&dev_info_req), sizeof(dev_info_req),
        reinterpret_cast<uint8_t *>(&dev_info_reply), sizeof(dev_info_reply)
    );

    // Linha 192 real:
    if (dev_info_reply.type != 0x05) {
        PX4_ERR("no devices found");
        return -EIO;
    }

    // Linhas 194-200 reais — verificação de CRC:
    uint8_t *reply_raw = reinterpret_cast<uint8_t *>(&dev_info_reply);
    uint8_t crc_calc = crc8(reinterpret_cast<uint8_t *>(&dev_info_reply.type),
                            dev_info_reply.length - 1);
    uint8_t crc_recv = reply_raw[dev_info_reply.length];

    if (crc_recv != crc_calc) {
        PX4_ERR("CRC error: got %02x, should be %02x", (int)crc_recv, (int)crc_calc);
        return -EIO;
    }

    // ============================================================
    // LINHA 208 DO ARQUIVO REAL — O BUG:
    // dev_name_len = 244, dev_name[32] → escrita em dev_name[244]
    // ============================================================
    dev_info_reply.payload.dev_name[dev_info_reply.payload.dev_name_len] = '\0';

    PX4_DEBUG("device info: hardware ID: 0x%08X, firmware ID: 0x%04X, device name: %s",
              (int)dev_info_reply.payload.hw_id,
              (int)dev_info_reply.payload.fw_id,
              dev_info_reply.payload.dev_name);

    int _retries = 1; (void)_retries;

    return OK;
}

int main()
{
    printf("=== Replay do codigo real: bst.cpp::BST::probe() ===\n");
    printf("Contraexemplo ESBMC: dev_name_len=244, dev_name[32]\n");
    printf("Esperado: heap-buffer-overflow na linha 208 do bst.cpp\n\n");

    // Aloca no heap para deteccao precisa pelo AddressSanitizer
    // (no PX4 real, o objeto BST e alocado no heap pelo modulo I2CSPIDriver)
    BSTPacket<BSTDeviceInfoReply> *reply_heap =
        (BSTPacket<BSTDeviceInfoReply>*)malloc(sizeof(BSTPacket<BSTDeviceInfoReply>));
    memset(reply_heap, 0, sizeof(BSTPacket<BSTDeviceInfoReply>));

    // Injeta o contraexemplo diretamente na struct heap-alocada
    reply_heap->type = 0x05;
    reply_heap->length = (uint8_t)(sizeof(BSTPacket<BSTDeviceInfoReply>) - 1);
    reply_heap->payload.dev_name_len = 244;

    printf("sizeof(BSTDeviceInfoReply) = %zu\n", sizeof(BSTDeviceInfoReply));
    printf("sizeof(dev_name) = %zu (indices validos: 0..31)\n",
           sizeof(reply_heap->payload.dev_name));
    printf("dev_name_len = %d\n", reply_heap->payload.dev_name_len);
    printf("\nExecutando linha 208 do bst.cpp real:\n");
    printf("  dev_info_reply.payload.dev_name[%d] = '\\0';\n\n",
           reply_heap->payload.dev_name_len);

    // LINHA 208 DO bst.cpp REAL — verbatim:
    reply_heap->payload.dev_name[reply_heap->payload.dev_name_len] = '\0';

    printf("Se chegou aqui: bug nao detectado (UB silencioso)\n");
    free(reply_heap);

    // Rodando probe() completo com stub I2C:
    printf("\n=== Rodando probe() completo com stub I2C ===\n");
    int ret = bst_probe_real();
    printf("probe() retornou: %d\n", ret);

    return 0;
}
