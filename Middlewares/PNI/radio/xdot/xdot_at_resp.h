#ifndef XDOT_AT_RESP_H
#define XDOT_AT_RESP_H

#include <stdint.h>
#include "radio.h"
#include "xdot_at_cmd.h"

#define XDOT_AT_RESP_HASH_INIT (5464)

#define XDOT_AT_RESP_STR_OK "OK"
#define XDOT_AT_RESP_STR_ERROR "ERROR"
#define XDOT_AT_RESP_STR_FREQ_US915 "US915"
#define XDOT_AT_RESP_STR_FREQ_EU868 "EU868"

typedef struct xdot_at_resp {
    uint8_t data[PNI_RADIO_BUFFER_RX_LEN];
    uint16_t count;
    uint32_t hash;
    PNI_RadioRet ret;
} Xdot_AtResp;

typedef enum xdot_at_resp_id {
    XDOT_AT_RESP_UNKNOWN = 0,
    XDOT_AT_RESP_ERROR,
    XDOT_AT_RESP_OK,
    XDOT_AT_RESP_AT_CMD_RECV,
    XDOT_AT_RESP_FREQ_US915,
    XDOT_AT_RESP_FREQ_EU868,
    XDOT_AT_RESP_MAX,
} Xdot_AtResponseId;

uint32_t xdot_at_resp_hash_char(const uint16_t s, uint32_t hash);
uint32_t xdot_at_resp_hash_str(const char *s);
int xdot_at_resp_hash_search(uint32_t hash);
int xdot_at_resp_hash_init(void);

#define xdot_at_resp_reset(resp) (memset(resp, 0, sizeof(*resp)))

#endif /* XDOT_AT_RESP_H */
