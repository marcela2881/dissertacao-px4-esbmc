// Modelo fiel de GPS::dumpGpsData() -- PX4 commit 8be2590c15,
// src/drivers/gps/gps.cpp linhas 727-761; struct de msg/GpsDump.msg.
// Mantidos os tipos reais: len do laco em size_t, sizeof(dump_data->data) = 79,
// dump_data->len em uint8_t. Stubs: hrt_absolute_time() e publish() (sem efeito
// sobre len/data).
#include <assert.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
// Assuncao portavel: CBMC e ESBMC aceitam a forma SV-COMP __VERIFIER_assume.
extern "C" void __VERIFIER_assume(int);
#define ASSUME(c) __VERIFIER_assume(c)
uint8_t nondet_uint8_t();
size_t nondet_size_t();
bool nondet_bool();
uint64_t nondet_u64();

struct gps_dump_s {            // msg/GpsDump.msg
    uint64_t timestamp;
    uint8_t  instance;
    uint32_t device_id;
    uint8_t  len;              // MSB = mensagem para o dispositivo
    uint8_t  data[79];
};
static uint64_t hrt_absolute_time(void) { return nondet_u64(); }
static void publish(const struct gps_dump_s *d) { (void)d; }

// Corpo de UMA iteracao do laco "while (len > 0)" -- copia literal da producao.
static void loop_body(const uint8_t **data, size_t *len, struct gps_dump_s *dump_data,
                      bool msg_to_gps_device)
{
    size_t write_len = *len;

    if (write_len > sizeof(dump_data->data) - dump_data->len) {
        write_len = sizeof(dump_data->data) - dump_data->len;
    }

    memcpy(dump_data->data + dump_data->len, *data, write_len);
    *data += write_len;
    dump_data->len += write_len;
    *len -= write_len;

    if (dump_data->len >= sizeof(dump_data->data)) {
        if (msg_to_gps_device) {
            dump_data->len |= 1 << 7;
        }

        dump_data->timestamp = hrt_absolute_time();
        publish(dump_data);
        dump_data->len = 0;
    }
}
