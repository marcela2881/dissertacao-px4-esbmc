/**
 * TESTE ESBMC - MAVLINK BUFFER OVERFLOW (FOCADO)
 * Versão otimizada para detectar vulnerabilidades críticas
 */

#include <cstring>
#include <cassert>
#include <cstdint>

#define BUFFER_SIZE 280

class Mavlink {
private:
    uint8_t _buf[BUFFER_SIZE];
    unsigned _buf_fill;
    bool _tx_buffer_low;

public:
    Mavlink() : _buf_fill(0), _tx_buffer_low(false) {
        // Inicialização simples sem loop
        memset(_buf, 0, BUFFER_SIZE);
    }
    
    // Função crítica do PX4 - mavlink_main.cpp
    void send_bytes(const uint8_t *buf, unsigned packet_len) {
        if (!_tx_buffer_low) {
            // VULNERABILIDADE: Esta condição pode falhar
            if (_buf_fill + packet_len < BUFFER_SIZE) {
                // CRÍTICO: memcpy sem validação robusta
                memcpy(&_buf[_buf_fill], buf, packet_len);
                _buf_fill += packet_len;
            } else {
                _tx_buffer_low = true;
            }
        }
    }
    
    unsigned get_buf_fill() const { return _buf_fill; }
    bool is_tx_buffer_low() const { return _tx_buffer_low; }
};

extern "C" {
    unsigned __VERIFIER_nondet_uint();
}

int main() {
    Mavlink mavlink;
    
    // Buffer de teste estático (sem loop)
    uint8_t test_data[BUFFER_SIZE] = {0};
    
    // Entrada não-determinística
    unsigned packet_len = __VERIFIER_nondet_uint();
    
    // Condições que podem causar overflow
    __ESBMC_assume(packet_len > 0);
    __ESBMC_assume(packet_len <= BUFFER_SIZE + 50); // Permite overflow
    
    // TESTE PRINCIPAL
    mavlink.send_bytes(test_data, packet_len);
    
    // PROPRIEDADES CRÍTICAS DE SEGURANÇA
    
    // P1: Buffer fill nunca deve exceder o tamanho físico
    assert(mavlink.get_buf_fill() <= BUFFER_SIZE);
    
    // P2: Se houve overflow, buffer_low deve estar ativo
    if (packet_len >= BUFFER_SIZE) {
        assert(mavlink.is_tx_buffer_low() || mavlink.get_buf_fill() == 0);
    }
    
    // TESTE ADICIONAL: Segunda operação
    unsigned second_packet = __VERIFIER_nondet_uint();
    __ESBMC_assume(second_packet > 0);
    __ESBMC_assume(second_packet <= BUFFER_SIZE);
    
    mavlink.send_bytes(test_data, second_packet);
    
    // P3: Após operações múltiplas, integridade mantida
    assert(mavlink.get_buf_fill() <= BUFFER_SIZE);
    
    return 0;
}