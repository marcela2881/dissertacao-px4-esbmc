/**
 * @file test_iridiumsbd_write.cpp
 *
 * MÓDULO: src/drivers/telemetry/iridiumsbd/IridiumSBD.cpp
 * FUNÇÃO: IridiumSBD::write() — escrita de pacote MAVLink no buffer de TX
 * MÉTODO: Bounded Model Checking com ESBMC / CBMC
 *
 * ============================================================
 * CONTEXTO DE VOO (resposta à crítica Barreto: "impacto no contexto de voo")
 * ============================================================
 * O driver IridiumSBD implementa comunicação via satélite Iridium SBD
 * (Short Burst Data), usado em missões de longo alcance em áreas remotas
 * como a Amazônia (projeto SAGRES), onde não há cobertura de rádio
 * convencional. É o link de telemetria de último recurso em campo.
 *
 * QUANDO pode ocorrer em voo:
 *   - A função write() é chamada repetidamente pelo stack MAVLink durante
 *     toda a missão, sempre que há telemetria a transmitir via satélite
 *   - Pacotes MAVLink fragmentados (enviados em múltiplos chunks menores
 *     que o tamanho total declarado no header) são o caso de uso normal
 *     pra mensagens longas — o bug é disparado por esse padrão comum
 *   - Em voo sobre floresta densa com interferência, o stack MAVLink
 *     pode reenviar fragmentos fora de ordem, aumentando a chance de
 *     um fragmento de tamanho buflen > _packet_length chegar
 *
 * IMPACTO no drone se disparado:
 *   - _packet_length é uint16_t: se buflen > _packet_length, a subtração
 *     "wraps around" para ~65535, fazendo _packet_length ficar gigante
 *   - Na próxima chamada, a guarda "SATCOM_TX_BUF_LEN - _tx_buf_write_idx
 *     - _packet_length < 0" falha silenciosamente (aritmética sinalizada
 *     com uint16_t grande não detecta o underflow como esperado)
 *   - O memcpy subsequente escreve além de _tx_buf[340], corrompendo
 *     memória adjacente da classe IridiumSBD no heap
 *   - Consequência: corrupção do estado interno do driver de telemetria
 *     via satélite — perda completa do link de comunicação de emergência
 *     durante a missão sobre a Amazônia. O operador perde visibilidade do
 *     drone em área remota sem outro link disponível.
 *
 * CONTROLABILIDADE da entrada:
 *   - buflen vem do stack MAVLink (dado interno, mas dependente do
 *     tamanho real das mensagens de telemetria)
 *   - _packet_length é calculado do header MAVLink (buffer[1] = payload_len)
 *   - Se o MAVLink enviar um chunk maior que o payload declarado no header
 *     (cenário de bug no stack MAVLink, ou mensagem corrompida em trânsito),
 *     a subtração estoura. Não é preciso acesso físico — basta um pacote
 *     MAVLink malformado chegando ao driver.
 *
 * ============================================================
 * MECANISMO DO BUG (análise estática)
 * ============================================================
 * IridiumSBD::write() — trecho real (IridiumSBD.cpp linhas 626-680):
 *
 *   uint16_t _packet_length;   // persiste entre chamadas
 *
 *   if (!_writing_mavlink_packet) {
 *       // Para MAVLink 2: _packet_length = payload_len + 12
 *       _packet_length = buflen;
 *   }
 *
 *   // Guarda: verifica se cabe no buffer de TX
 *   if (SATCOM_TX_BUF_LEN - _tx_buf_write_idx - _packet_length < 0) {
 *       _tx_buf_write_idx = 0;
 *   }
 *
 *   // BUG: sem verificar se buflen <= _packet_length
 *   _packet_length -= buflen;   // uint16_t underflow se buflen > _packet_length
 *
 *   memcpy(_tx_buf + _tx_buf_write_idx, buffer, buflen);
 *   _tx_buf_write_idx += buflen;
 *
 * CWE-191 (Integer Underflow) → CWE-787 (Out-of-Bounds Write)
 */

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

extern size_t nondet_size_t();
extern int nondet_int();

// ================== CONSTANTES REAIS, extraídas de IridiumSBD.h ==================
#define SATCOM_TX_BUF_LEN     340   // IridiumSBD.h linha 88
#define SATCOM_MAX_MESSAGE_LENGTH 270

// ================== ESTADO REAL (membros relevantes da classe IridiumSBD) ==================
static uint8_t  _tx_buf[SATCOM_TX_BUF_LEN] = {};
static int      _tx_buf_write_idx = 0;
static uint16_t _packet_length = 0;      // uint16_t real — tipo do bug
static bool     _writing_mavlink_packet = false;

// ================== LÓGICA REAL EXTRAÍDA de IridiumSBD::write(), linhas 626-680 ==================
// Simplificação: modelamos apenas o caminho MAVLink 2 (byte[0] == 253),
// que é o protocolo em uso no SAGRES. O restante dos branches (MAVLink 1,
// pacotes genéricos) segue o mesmo padrão de subtração sem checagem.

ssize_t iridiumsbd_write(const char *buffer, size_t buflen)
{
    // Linha 629 real — validação inicial de tamanho total
    if ((ssize_t)buflen > SATCOM_TX_BUF_LEN) {
        return -1; // PX4_ERROR
    }

    // Parsing do tamanho do pacote (linhas 636-656 reais)
    if (!_writing_mavlink_packet) {
        if (buflen >= 2 && (unsigned char)buffer[0] == 253 && buflen == 10) {
            // MAVLink 2 header — payload_len está em buffer[1]
            const uint8_t payload_len = (uint8_t)buffer[1];
            _packet_length = payload_len + 12;
            _writing_mavlink_packet = true;
        } else {
            _packet_length = (uint16_t)buflen;
        }
    }

    // Linha 661 real — guarda de espaço disponível
    if (SATCOM_TX_BUF_LEN - _tx_buf_write_idx - _packet_length < 0) {
        _tx_buf_write_idx = 0;
    }

    // Linha 667 real — BUG: sem verificar se buflen <= _packet_length
    // Se buflen > _packet_length, uint16_t wraps around para ~65535
    _packet_length -= (uint16_t)buflen;

    if (_packet_length == 0) {
        _writing_mavlink_packet = false;
    }

    // Linha 675 real — memcpy sem verificação adicional
    memcpy(_tx_buf + _tx_buf_write_idx, buffer, buflen);
    _tx_buf_write_idx += (int)buflen;

    return (ssize_t)buflen;
}

int main()
{
    // Cenário: primeira chamada com header MAVLink 2 (10 bytes, payload_len pequeno)
    char header[10] = {(char)253, 5, 0, 0, 0, 0, 0, 0, 0, 0}; // payload_len = 5
    iridiumsbd_write(header, 10);
    // Após essa chamada: _packet_length = 5+12-10 = 7, _writing_mavlink_packet = true

    // Segunda chamada: fragmento com buflen > _packet_length restante
    size_t second_buflen = nondet_size_t();
    __VERIFIER_assume(second_buflen > 0 && second_buflen <= SATCOM_TX_BUF_LEN);
    __VERIFIER_assume(second_buflen > _packet_length); // força o underflow

    char second_buf[SATCOM_TX_BUF_LEN];
    memset(second_buf, 0, sizeof(second_buf));

    iridiumsbd_write(second_buf, second_buflen);

    // PROPRIEDADE: _tx_buf_write_idx nunca deve ultrapassar o tamanho real do buffer.
    // Qualquer escrita além de _tx_buf[SATCOM_TX_BUF_LEN-1] é CWE-787.
    assert(_tx_buf_write_idx <= SATCOM_TX_BUF_LEN);

    return 0;
}
