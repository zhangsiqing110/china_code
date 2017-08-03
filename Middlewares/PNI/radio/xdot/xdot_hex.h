#ifndef XDOT_HEX_H
#define XDOT_HEX_H

int xdot_hex_to_bytes(const uint8_t *hex, uint16_t hex_size, uint8_t *out,
    uint16_t out_size);

void xdot_bytes_to_hex(const uint8_t *bytes, uint16_t size, char *hex,
        const char delim, uint8_t upper);

#endif /* XDOT_HEX_H */
