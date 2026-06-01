#include <assert.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

int nondet_int();
uint8_t nondet_uint8_t();
unsigned int nondet_uint();

#define GPS_DUMP_DATA_SIZE 200

struct gps_dump_s {
    uint8_t data[GPS_DUMP_DATA_SIZE];
    uint8_t len;
    uint8_t instance;
};

void dumpGpsData(uint8_t *data, unsigned int len, gps_dump_s *dump_data) {
    if (!dump_data) return;
    while (len > 0) {
        unsigned int write_len = len;
        if (write_len > GPS_DUMP_DATA_SIZE - dump_data->len) {
            write_len = GPS_DUMP_DATA_SIZE - dump_data->len;
        }
        memcpy(dump_data->data + dump_data->len, data, write_len);
        data += write_len;
        dump_data->len += write_len;
        len -= write_len;
        if (dump_data->len >= GPS_DUMP_DATA_SIZE) {
            dump_data->len = 0;
        }
    }
}

int main() {
    unsigned int input_len = nondet_uint();
    __CPROVER_assume(input_len > 0 && input_len <= 300);

    uint8_t input_data[300];
    gps_dump_s dump_buffer;
    dump_buffer.len = nondet_uint8_t();
    __CPROVER_assume(dump_buffer.len < GPS_DUMP_DATA_SIZE);

    dumpGpsData(input_data, input_len, &dump_buffer);

    assert(dump_buffer.len <= GPS_DUMP_DATA_SIZE);
    return 0;
}
