/*
 * ============================================================================
 * FALSO POSITIVO (v5). NAO USAR COMO EVIDENCIA DE DEFEITO.
 * Este arquivo omite duas guardas que o bst.cpp ja tinha no commit testado
 * (8be2590c15), vindas do commit e8e86a2e0f (marco de 2026). Ver
 * harness_fiel.cpp e LEIA-ME.txt desta pasta. Mantido so para rastreio.
 * ============================================================================
 */
/**
 * @file test_bst_dev_name.cpp
 *
 * MÓDULO: src/drivers/telemetry/bst/bst.cpp
 * FUNÇÃO: BST::probe() — inicialização do link de telemetria Black Sheep
 * MÉTODO: Bounded Model Checking com ESBMC / CBMC
 *
 * ============================================================
 * CONTEXTO DE VOO (resposta à crítica Barreto: "impacto no contexto de voo")
 * ============================================================
 * O driver BST implementa o protocolo Black Sheep Telemetry, usado para
 * transmitir dados de posição, atitude e bateria do drone para um controle
 * de rádio TBS Crossfire. É inicializado uma vez durante o boot do PX4,
 * antes de armar o drone. O bug ocorre durante a chamada BST::probe(), que
 * descobre o dispositivo BST conectado via I2C e lê sua identificação.
 *
 * QUANDO pode ocorrer em voo:
 *   - Durante o boot do sistema, antes de qualquer armamento
 *   - Se o dispositivo BST for desconectado e reconectado em voo (hot-swap),
 *     o probe pode ser chamado novamente
 *   - Se um dispositivo I2C malicioso (ou defeituoso) for conectado no
 *     mesmo barramento com endereço 0x76 (BST_ADDR)
 *
 * IMPACTO no drone se disparado:
 *   - A escrita além de dev_name[32] sobrescreve bytes adjacentes na struct
 *     BSTDeviceInfoReply dentro do BSTPacket, que compartilha o stack frame
 *     de BST::probe(). Em hardware sem MMU (Pixhawk/NuttX), isso pode
 *     corromper variáveis locais do stack, incluindo o ponteiro de retorno
 *     da função — potencial execução de código arbitrário ou crash do
 *     processo de inicialização, impedindo o drone de armar.
 *   - No melhor caso: probe() retorna erro, BST não inicializa, telemetria
 *     indisponível durante a missão (perda de dados de monitoramento ambiental
 *     SAGRES).
 *   - No pior caso: corrupção de memória adjacente no stack frame, com
 *     consequências não determinísticas no sistema de voo.
 *
 * CONTROLABILIDADE da entrada:
 *   - dev_name_len vem diretamente do dispositivo I2C via protocolo BST
 *   - Um dispositivo BST defeituoso, clone não autorizado, ou adversário
 *     com acesso físico ao barramento I2C pode enviar dev_name_len > 31
 *   - O CRC é verificado DEPOIS da leitura (linha 202-204 do bst.cpp),
 *     mas a escrita em dev_name[dev_name_len] (linha 208) não tem checagem
 *     de limite — o CRC não protege contra essa exploração
 *
 * ============================================================
 * MECANISMO DO BUG (análise estática)
 * ============================================================
 * struct BSTDeviceInfoReply {
 *     uint32_t hw_id;          // 4 bytes
 *     uint16_t fw_id;          // 2 bytes
 *     uint8_t  dev_name_len;   // 1 byte — VINDO DO DISPOSITIVO I2C
 *     char     dev_name[32];   // 32 bytes — índices válidos: 0..31
 * };
 *
 * Linha 208 do bst.cpp (real):
 *   dev_info_reply.payload.dev_name[dev_info_reply.payload.dev_name_len] = '\0';
 *
 * dev_name_len é uint8_t → pode valer 0 a 255.
 * dev_name[32] → índices válidos: 0 a 31.
 * Se dev_name_len = 32..255 → escrita fora dos limites do array (CWE-787).
 * Classificação: CWE-1284 (validação imprópria de quantidade especificada)
 *                propagando para CWE-787 (escrita fora dos limites).
 */

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

extern uint8_t nondet_uint8();

// ================== ESTRUTURAS REAIS, extraídas de bst.cpp ==================
#pragma pack(push, 1)

struct BSTDeviceInfoReply {
    uint32_t hw_id;          // 4 bytes
    uint16_t fw_id;          // 2 bytes
    uint8_t  dev_name_len;   // 1 byte — vem do dispositivo I2C, não confiável
    char     dev_name[32];   // 32 bytes — índices válidos: 0..31
};

// BSTPacket<BSTDeviceInfoReply> — wrapper real em torno do payload
struct BSTPacket_DeviceInfoReply {
    uint8_t length;
    uint8_t type;
    BSTDeviceInfoReply payload;
    uint8_t crc;
};

#pragma pack(pop)

// ================== LÓGICA REAL EXTRAÍDA de bst.cpp::probe(), linha 208 ==================
// O código real (após verificação de CRC):
//
//   dev_info_reply.payload.dev_name[dev_info_reply.payload.dev_name_len] = '\0';
//
// Não existe nenhuma checagem de "dev_name_len < sizeof(dev_name)" antes dessa linha.
// Reproduzido abaixo verbatim, com o dispositivo I2C modelado por nondet.

void bst_probe(BSTPacket_DeviceInfoReply *dev_info_reply)
{
    // Simulação do recebimento via I2C: o payload completo vem do dispositivo
    // externo — todos os campos, incluindo dev_name_len, são não confiáveis.
    // O PX4 real faz um transfer() I2C que preenche toda a struct de uma vez.
    // Modelamos só o campo crítico como nondet; o restante é zero-inicializado.
    dev_info_reply->payload.dev_name_len = nondet_uint8(); // dado externo, 0..255

    // Linha 208 real de bst.cpp — sem checagem de limite:
    dev_info_reply->payload.dev_name[dev_info_reply->payload.dev_name_len] = '\0';
}

int main()
{
    BSTPacket_DeviceInfoReply dev_info_reply;
    memset(&dev_info_reply, 0, sizeof(dev_info_reply));

    bst_probe(&dev_info_reply);

    // PROPRIEDADE: o índice usado nunca deve ultrapassar o tamanho real do array.
    // dev_name[32] tem índices válidos 0..31; qualquer acesso em [32]..[255]
    // é escrita fora dos limites — CWE-787.
    assert(dev_info_reply.payload.dev_name_len <= 31);

    return 0;
}
