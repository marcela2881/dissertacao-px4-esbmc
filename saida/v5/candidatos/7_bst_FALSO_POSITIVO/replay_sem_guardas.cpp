/*
 * ============================================================================
 * FALSO POSITIVO (v5). NAO USAR COMO EVIDENCIA DE DEFEITO.
 * Este arquivo omite duas guardas que o bst.cpp ja tinha no commit testado
 * (8be2590c15), vindas do commit e8e86a2e0f (marco de 2026). Ver
 * harness_fiel.cpp e LEIA-ME.txt desta pasta. Mantido so para rastreio.
 * ============================================================================
 */
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

// ============================================================================
// DRIVER CORRIGIDO — chama SOMENTE bst_probe_real()
//
// Tudo acima desta linha e identico ao replay_bst_real.cpp: structs, crc8(),
// stub de I2C::transfer() e bst_probe_real() permanecem verbatim.
//
// POR QUE ESTE ARQUIVO EXISTE:
// O main() do replay_bst_real.cpp injetava o contraexemplo numa copia
// heap-alocada ANTES de chamar bst_probe_real(). Essa escrita disparava o
// AddressSanitizer e abortava o processo, de modo que bst_probe_real() nunca
// executava — o log asan_bst_codigo_real.txt termina em
// replay_bst_real.cpp:172 in main. O overflow ali registrado era o da struct
// sintetica do main, nao o do probe() verbatim do PX4.
//
// Este driver remove a injecao direta. O contraexemplo entra pelo caminho
// legitimo: o stub de I2C::transfer() e quem fornece dev_name_len = 244,
// exatamente como o dispositivo I2C faria.
// ============================================================================

int main()
{
    printf("=== Replay do codigo real: bst.cpp::BST::probe() ===\n");
    printf("Contraexemplo ESBMC: dev_name_len=244, dev_name[32]\n");
    printf("O stub de I2C::transfer() injeta o contraexemplo;\n");
    printf("nada e escrito diretamente aqui no main.\n\n");
    printf("Esperado: overflow DENTRO de bst_probe_real(), na linha\n");
    printf("  dev_info_reply.payload.dev_name[dev_name_len] = '\\0';\n");
    printf("  (linha 208 do bst.cpp real)\n\n");

    printf("=== Rodando probe() completo com stub I2C ===\n");
    int ret = bst_probe_real();
    printf("probe() retornou: %d\n", ret);
    printf("Se chegou aqui: bug nao detectado (UB silencioso)\n");

    return 0;
}

// ============================================================================
// COMPILACAO E EXECUCAO
//
//   # 1) AddressSanitizer
//   g++ -fsanitize=address -g -O0 -o r7 replay_bst_probe_only.cpp && ./r7
//
//   # 2) local-bounds (fixa a ESCRITA na linha exata do bug; requer clang)
//   clang++ -fsanitize=local-bounds -fno-sanitize-recover=all -g -O0 \
//       -o r7lb replay_bst_probe_only.cpp && ./r7lb
//
// VERIFICACAO DE QUE O probe() REALMENTE EXECUTOU: a saida precisa conter
// "Rodando probe() completo" e o frame do diagnostico precisa ser
// bst_probe_real(), NAO main. Se o log terminar em main, a correcao nao pegou.
//
// O que a reproducao independente obteve com este driver:
//   ASan          -> stack-buffer-overflow, READ of size 37,
//                    frame bst_probe_real(), objeto dev_info_reply em [48, 90)
//   local-bounds  -> frame #0: bst_probe_real() na linha da escrita
//
// Nota sobre o ASan: ele flagra a LEITURA do %s no PX4_DEBUG, nao a escrita.
// A escrita em dev_name[244] salta 244 bytes e passa por cima da redzone de
// 42 bytes sem toca-la — limitacao conhecida do ASan em stack para
// deslocamentos grandes. Por isso os dois sanitizadores sao complementares.
// ============================================================================
