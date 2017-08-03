#include <stdint.h>
#include "radio.h"
#include "xdot_at_cmd.h"
#include "xdot_at_resp.h"

static uint32_t m_xdot_at_resp_hashes[XDOT_AT_RESP_MAX];

uint32_t xdot_at_resp_hash_char(const uint16_t s, uint32_t hash)
{
    return ((hash << 2) + hash) + s;
}

uint32_t xdot_at_resp_hash_str(const char *s)
{
    char c = 0;
    uint32_t hash = XDOT_AT_RESP_HASH_INIT;

    while ((c = *(s++))) {
        hash = xdot_at_resp_hash_char(c, hash);
    }
    return hash;
}

static void xdot_at_resp_hash_add(const int id, const char *s)
{
    if ((id > 0) && (id < XDOT_AT_RESP_MAX)) {
        m_xdot_at_resp_hashes[id] = xdot_at_resp_hash_str(s);
    }
}

int xdot_at_resp_hash_search(uint32_t hash)
{
    int i;
    for (i = 0; i < XDOT_AT_RESP_MAX; i++) {
        if (m_xdot_at_resp_hashes[i] == hash) {
            return i;
        }
    }
    return XDOT_AT_RESP_UNKNOWN;
}

int xdot_at_resp_hash_init(void)
{
    xdot_at_resp_hash_add(XDOT_AT_RESP_OK, "OK");
    xdot_at_resp_hash_add(XDOT_AT_RESP_ERROR, "ERROR");
    xdot_at_resp_hash_add(XDOT_AT_RESP_AT_CMD_RECV, XDOT_AT_CMD_RX_ONCE);
    xdot_at_resp_hash_add(XDOT_AT_RESP_FREQ_US915,
            XDOT_AT_RESP_STR_FREQ_US915);
    xdot_at_resp_hash_add(XDOT_AT_RESP_FREQ_EU868,
            XDOT_AT_RESP_STR_FREQ_EU868);
    return 0;
}



