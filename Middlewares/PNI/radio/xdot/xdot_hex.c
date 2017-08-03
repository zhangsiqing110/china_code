#include <stdint.h>
#include "radio.h"

static const char *HEX_CHARS = "0123456789abcdef0123456789ABCDEF";

static int xdot_hex_to_nibble(const uint8_t hex)
{
    if ((hex >= '0') && (hex <= '9')) {
        return (hex - '0');
    } else if ((hex >= 'a') && (hex <= 'f')) {
        return (hex - 'a' + 10);
    } else if ((hex >= 'A') && (hex <= 'F')) {
        return (hex - 'A' + 10);
    } else {
        return PNI_RADIO_RET_EINVAL;
    }
}

int xdot_hex_to_bytes(const uint8_t *hex, uint16_t hex_size, uint8_t *out,
    uint16_t out_size)
{
    int count = 0;
    int hi = 0, lo = 0;

    while ((hex_size) && (out_size)) {
        hi = xdot_hex_to_nibble(*hex++);
        lo = xdot_hex_to_nibble(*hex++);
        if ((hi < 0) || (lo < 0)) {
            return PNI_RADIO_RET_EINVAL;
        }
        *out++ = (hi << 4) | lo;
        count++;
        hex_size -= 2;
        out_size --;
    }
    return count;
}

void xdot_bytes_to_hex(const uint8_t *bytes, uint16_t size, char *hex,
        const char delim, uint8_t upper)
{
    uint8_t byte = 0;
    char *c = hex;
    while (size--) {
        byte = *bytes;
        if (upper) {
            c[0] = HEX_CHARS[((byte >> 4) & 0xF) + 16];
            c[1] = HEX_CHARS[(byte & 0x0F) + 16];
        } else {
            c[0] = HEX_CHARS[(byte >> 4) & 0xF];
            c[1] = HEX_CHARS[(byte & 0x0F)];
        }
        c[2] = delim;
        bytes++;
        c += 3;
    }
    c[-1] = 0;
}
