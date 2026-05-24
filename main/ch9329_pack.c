#include "ch9329_pack.h"

int pack_ch9329_data(CH9329CMD cmd, const uint8_t *data, int length, uint8_t *out) {
    if (length < 0 || length > CH9329_MAX_PAYLOAD) {
        return 0;
    }

    out[0] = 0x57;
    out[1] = 0xAB;
    out[2] = 0x00;
    out[3] = (uint8_t)cmd;
    out[4] = (uint8_t)length;
    for (int i = 0; i < length; ++i) {
        out[5 + i] = data[i];
    }

    int sum_pos = length + CH9329_HEADER_LEN - 1;
    uint8_t sum = 0;
    for (int i = 0; i < sum_pos; ++i) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[sum_pos] = sum;

    return length + CH9329_HEADER_LEN;
}
