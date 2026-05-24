#ifndef CH9329_PACK_H_
#define CH9329_PACK_H_

#include <stdint.h>

#define CH9329_HEADER_LEN  6
#define CH9329_MAX_PAYLOAD 64

typedef enum {
    CMD_GET_INFO              = 0x01,
    CMD_SEND_KB_GENERAL_DATA  = 0x02,
    CMD_SEND_KB_MEDIA_DATA    = 0x03,
} CH9329CMD;

// Writes header + payload + checksum to `out` (caller-owned, at least
// CH9329_HEADER_LEN + CH9329_MAX_PAYLOAD bytes). Returns total bytes
// written on success, or 0 if length is out of range.
int pack_ch9329_data(CH9329CMD cmd, const uint8_t *data, int length, uint8_t *out);

#endif
