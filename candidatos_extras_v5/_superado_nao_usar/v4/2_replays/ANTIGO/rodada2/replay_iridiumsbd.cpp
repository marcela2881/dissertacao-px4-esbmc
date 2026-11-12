#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#define SATCOM_TX_BUF_LEN 340
static uint8_t _tx_buf[SATCOM_TX_BUF_LEN];
static int _tx_buf_write_idx = 0;
static uint16_t _packet_length = 0;
static int _writing_mavlink_packet = 0;
int iridiumsbd_write(const char *buffer, size_t buflen) {
    if ((int)buflen > SATCOM_TX_BUF_LEN) return -1;
    if (!_writing_mavlink_packet) {
        if (buflen >= 2 && (unsigned char)buffer[0] == 253 && buflen == 10) {
            _packet_length = (uint8_t)buffer[1] + 12;
            _writing_mavlink_packet = 1;
        } else {
            _packet_length = (uint16_t)buflen;
        }
    }
    if (SATCOM_TX_BUF_LEN - _tx_buf_write_idx - _packet_length < 0)
        _tx_buf_write_idx = 0;
    _packet_length -= (uint16_t)buflen;
    if (_packet_length == 0) _writing_mavlink_packet = 0;
    memcpy(_tx_buf + _tx_buf_write_idx, buffer, buflen);
    _tx_buf_write_idx += (int)buflen;
    return (int)buflen;
}
int main() {
    char header[10] = {(char)253, 5, 0, 0, 0, 0, 0, 0, 0, 0};
    iridiumsbd_write(header, 10);
    printf("Apos header: _packet_length=%d _tx_buf_write_idx=%d\n",
           _packet_length, _tx_buf_write_idx);
    char second[334];
    memset(second, 'A', sizeof(second));
    printf("Enviando segundo chunk de 334 bytes (> _packet_length=%d)...\n", _packet_length);
    iridiumsbd_write(second, 334);
    printf("_tx_buf_write_idx=%d SATCOM_TX_BUF_LEN=%d\n",
           _tx_buf_write_idx, SATCOM_TX_BUF_LEN);
    return 0;
}
