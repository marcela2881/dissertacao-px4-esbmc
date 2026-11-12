/**
 * @file harness_fiel.cpp — candidato 7 (bst.cpp), conferencia contra producao
 *
 * MODULO PX4: src/drivers/telemetry/bst/bst.cpp, BST::probe()
 * BASE: PX4 commit 8be2590c15 (04/06/2026), linhas 187-221
 *
 * POR QUE ESTE ARQUIVO EXISTE (v5)
 * O harness_sem_guardas.cpp (antigo test_bst_dev_name.cpp) dizia "Nao existe
 * nenhuma checagem de dev_name_len < sizeof(dev_name)". Isso e falso para o
 * commit testado: desde o commit e8e86a2e0f (marco de 2026, "validate reply
 * length and dev_name_len before use"), o probe() tem DUAS guardas:
 *
 *   if (dev_info_reply.length >= sizeof(dev_info_reply)) { ... return -EIO; }
 *   ...
 *   if (dev_info_reply.payload.dev_name_len >= sizeof(dev_info_reply.payload.dev_name)) {
 *       dev_info_reply.payload.dev_name_len = sizeof(dev_info_reply.payload.dev_name) - 1;
 *   }
 *
 * O harness antigo omitia as duas. Pela classificacao da dissertacao,
 * guarda de producao omitida no harness = FALSO POSITIVO.
 *
 * Este harness inclui as guardas. A resposta inteira do dispositivo I2C e
 * livre (nondet), e o resultado do CRC tambem (sobreaproximacao: qualquer
 * resposta pode ou nao passar no CRC). Resultado esperado: SUCCESSFUL.
 */

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

extern uint8_t nondet_uint8();
extern bool nondet_bool();

#pragma pack(push, 1)
struct BSTDeviceInfoReply {
	uint32_t hw_id;
	uint16_t fw_id;
	uint8_t dev_name_len;
	char dev_name[32];
};

struct BSTPacket_DeviceInfoReply {   // BSTPacket<BSTDeviceInfoReply>
	uint8_t length;
	uint8_t type;
	BSTDeviceInfoReply payload;
	uint8_t crc;
};
#pragma pack(pop)

#define EIO 5

// ================== LOGICA REAL, BST::probe() (8be2590c15, linhas 187-221) ==================
int bst_probe(BSTPacket_DeviceInfoReply &dev_info_reply)
{
	// send_packet(): o dispositivo I2C preenche a resposta inteira
	uint8_t *raw = reinterpret_cast<uint8_t *>(&dev_info_reply);

	for (size_t i = 0; i < sizeof(dev_info_reply); i++) {
		raw[i] = nondet_uint8();
	}

	if (dev_info_reply.type != 0x05) {
		return -EIO;
	}

	uint8_t *reply_raw = reinterpret_cast<uint8_t *>(&dev_info_reply);

	if (dev_info_reply.length >= sizeof(dev_info_reply)) {
		return -EIO;
	}

	// crc8(&type, length - 1): CRC abstraido como nondet; a leitura abaixo e a real
	uint8_t crc_recv = reply_raw[dev_info_reply.length];
	(void)crc_recv;
	bool crc_ok = nondet_bool();

	if (!crc_ok) {
		return -EIO;
	}

	if (dev_info_reply.payload.dev_name_len >= sizeof(dev_info_reply.payload.dev_name)) {
		dev_info_reply.payload.dev_name_len = sizeof(dev_info_reply.payload.dev_name) - 1;
	}

	dev_info_reply.payload.dev_name[dev_info_reply.payload.dev_name_len] = '\0';

	// PROPRIEDADE: o indice escrito e valido
	assert(dev_info_reply.payload.dev_name_len <= 31);

	return 0;
}

int main()
{
	BSTPacket_DeviceInfoReply dev_info_reply;
	memset(&dev_info_reply, 0, sizeof(dev_info_reply));
	bst_probe(dev_info_reply);
	return 0;
}
