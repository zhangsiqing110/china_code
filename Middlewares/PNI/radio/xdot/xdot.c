#include <stdint.h>
#include <string.h>

#include "radio.h"
#include "xdot.h"
#include "xdot_hex.h"
#include "xdot_at_cmd.h"
#include "xdot_at_resp.h"
#include "parking.h"
#include "parking_buffer.h"
#include "ble_clock.h"

static UART_HandleTypeDef *m_xdot_huart;

static char m_xdot_buff_tx[PNI_RADIO_BUFFER_TX_LEN] = { 0 };

#define XDOT_BUFF_W_PTR(h) ( \
        (PNI_RADIO_BUFFER_RX_LEN - h->hdmarx->Instance->CNDTR) \
        & (PNI_RADIO_BUFFER_RX_LEN - 1))

static uint8_t m_xdot_buff_rx[PNI_RADIO_BUFFER_RX_LEN] = { 0 };
static uint8_t m_xdot_buff_rx_idx = 0;

static uint8_t m_xdot_resp_data[PNI_RADIO_BUFFER_RX_LEN] = { 0 };
static uint8_t m_xdot_resp_data_idx = 0;
static uint32_t m_xdot_resp_hash = XDOT_AT_RESP_HASH_INIT;

static Xdot_IsmBand m_xdot_ism_band = XDOT_ISM_BAND_US915;

static const int XDOT_SF_DR_MAP[XDOT_ISM_BAND_MAX][XDOT_SF_MAX] = {
    [XDOT_ISM_BAND_EU868] = {
        [XDOT_SF_12_BW_125] = XDOT_DATA_RATE_DR0,
        [XDOT_SF_11_BW_125] = XDOT_DATA_RATE_DR1,
        [XDOT_SF_10_BW_125] = XDOT_DATA_RATE_DR2,
        [XDOT_SF_9_BW_125] = XDOT_DATA_RATE_DR3,
        [XDOT_SF_8_BW_125] = XDOT_DATA_RATE_DR4,
        [XDOT_SF_8_BW_500] = XDOT_DATA_RATE_INVALID,
        [XDOT_SF_7_BW_125] = XDOT_DATA_RATE_DR5,
        [XDOT_SF_7_BW_250] = XDOT_DATA_RATE_DR6,
        [XDOT_SF_FSK] = XDOT_DATA_RATE_DR7,
    },
    [XDOT_ISM_BAND_US915] = {
        [XDOT_SF_12_BW_125] = XDOT_DATA_RATE_INVALID,
        [XDOT_SF_11_BW_125] = XDOT_DATA_RATE_INVALID,
        [XDOT_SF_10_BW_125] = XDOT_DATA_RATE_DR0,
        [XDOT_SF_9_BW_125] = XDOT_DATA_RATE_DR1,
        [XDOT_SF_8_BW_125] = XDOT_DATA_RATE_DR2,
        [XDOT_SF_8_BW_500] = XDOT_DATA_RATE_DR4,
        [XDOT_SF_7_BW_125] = XDOT_DATA_RATE_DR3,
        [XDOT_SF_7_BW_250] = XDOT_DATA_RATE_INVALID,
        [XDOT_SF_FSK] = XDOT_DATA_RATE_INVALID,
    },
};

static const int XDOT_DR_PAYLOAD_MAP[XDOT_ISM_BAND_MAX][XDOT_DATA_RATE_MAX] = {
    [XDOT_ISM_BAND_EU868] = {
        [XDOT_DATA_RATE_DR0] = 51,
        [XDOT_DATA_RATE_DR1] = 51,
        [XDOT_DATA_RATE_DR2] = 51,
        [XDOT_DATA_RATE_DR3] = 115,
        [XDOT_DATA_RATE_DR4] = 242,
        [XDOT_DATA_RATE_DR5] = 242,
        [XDOT_DATA_RATE_DR6] = 242,
        [XDOT_DATA_RATE_DR7] = 50,
    },
    [XDOT_ISM_BAND_US915] = {
        [XDOT_DATA_RATE_DR0] = 11,
        [XDOT_DATA_RATE_DR1] = 53,
        [XDOT_DATA_RATE_DR2] = 129,
        [XDOT_DATA_RATE_DR3] = 242,
        [XDOT_DATA_RATE_DR4] = 242,
        [XDOT_DATA_RATE_DR5] = 0,
        [XDOT_DATA_RATE_DR6] = 0,
        [XDOT_DATA_RATE_DR7] = 0,
    },
};

static Xdot_State m_xdot_state = {
    .flags = XDOT_STATE_FLAG_NONE,
    .join_retry_count = 0,
    .tx_retry_count = 0,
};

static Xdot_AtCmdIface m_xdot_at_cmd_iface = { 0 };

static void xdot_wake(void)
{
    HAL_GPIO_WritePin(XDOT_WAKE_GPIO_Port, XDOT_WAKE_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(XDOT_WAKE_GPIO_Port, XDOT_WAKE_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
}

static int xdot_buffer_to_int(uint8_t *buffer, uint16_t count)
{
    int i;
    int ret = 0;

    for (i = 0; i < count; i++) {
        ret = (ret << 3) + (ret << 1) + (buffer[i]) - '0';
    }
    return ret;
}

static uint8_t xdot_buff_rx_get(void)
{
    uint8_t c = 0;
    if (m_xdot_buff_rx_idx != XDOT_BUFF_W_PTR(m_xdot_huart)) {
        c = m_xdot_buff_rx[m_xdot_buff_rx_idx++];
        m_xdot_buff_rx_idx &= (PNI_RADIO_BUFFER_RX_LEN - 1);
    }
    return c;
}

static int xdot_get_resp(Xdot_AtResp *resp)
{
    uint8_t b;
    int resp_id = 0;

    while (b = xdot_buff_rx_get()) {
        if (b == 0x0D) {
            // ignore CR
            continue;
        } else if (b == 0x0A) {
            // end command on LF
            resp_id = xdot_at_resp_hash_search(m_xdot_resp_hash);
            switch (resp_id) {

            case XDOT_AT_RESP_OK:
                resp->ret = PNI_RADIO_RET_OK;
                m_xdot_state.flags &= ~(XDOT_STATE_FLAG_RESP_WAIT);
                goto exit_done;

            case XDOT_AT_RESP_ERROR:
                resp->ret = PNI_RADIO_RET_EINVAL;
                m_xdot_state.flags &= ~(XDOT_STATE_FLAG_RESP_WAIT);
                goto exit_done;

            default:
                if (m_xdot_resp_data_idx) {
                    resp->count = m_xdot_resp_data_idx;
                    resp->hash = m_xdot_resp_hash;
                    memcpy(resp->data, m_xdot_resp_data, resp->count);
                }
                break;
            }
            m_xdot_resp_data_idx = 0;
            m_xdot_resp_hash = XDOT_AT_RESP_HASH_INIT;
            
        } else {
            m_xdot_resp_data[m_xdot_resp_data_idx++] = b;
            m_xdot_resp_data_idx %= PNI_RADIO_BUFFER_RX_LEN;
            m_xdot_resp_hash = xdot_at_resp_hash_char(b, m_xdot_resp_hash);
        }
    }

    return PNI_RADIO_RET_EBUSY;

exit_done:
    m_xdot_resp_data_idx = 0;
    m_xdot_resp_hash = 0;
    return resp->ret;
}

static int xdot_resp_wait(Xdot_AtResp *resp, int retries)
{
    int ret;
    while (retries--) {
        ret = xdot_get_resp(resp);
        if (ret != PNI_RADIO_RET_EBUSY) {
            XDOT_D("ret: %d, retries: %d", ret, retries);
            return ret;
        }
        Clock_Wait(1);
    }
    return PNI_RADIO_RET_EIO;
}

static int xdot_at_cmd_exec(char *buffer, uint16_t count)
{
    /* if sleeping, wake it up */
    if (m_xdot_state.flags & XDOT_STATE_FLAG_IN_SLEEP) {
        XDOT_I("waking up ...");
        m_xdot_state.flags &= ~(XDOT_STATE_FLAG_IN_SLEEP);
        xdot_wake();
    }

    // already waiting for a resp, return wait time
    if (m_xdot_state.flags & XDOT_STATE_FLAG_RESP_WAIT) {
        return PNI_RADIO_RET_EBUSY;
    }

    XDOT_D("cmd: %s", buffer);
    // set waiting for resp flag
    m_xdot_state.flags |= XDOT_STATE_FLAG_RESP_WAIT;

    if (HAL_UART_Transmit(m_xdot_huart, (uint8_t *)buffer, count,
            PNI_RADIO_UART_TIMEOUT) != HAL_OK) {
        XDOT_E();
        return PNI_RADIO_RET_EIO;
    }

    return PNI_RADIO_RET_OK;
}

static int xdot_set_wake_mode(Xdot_WakeMode mode)
{
    return xdot_at_cmd_set_wake_mode(&m_xdot_at_cmd_iface, mode);
}

static int xdot_get_sf_dr(Xdot_IsmBand ism_band, Xdot_SpreadingFactor sf)
{
    return XDOT_SF_DR_MAP[ism_band][sf];
}

static int xdot_set_tx_data_rate(Xdot_IsmBand ism_band, Xdot_SpreadingFactor sf)
{
    int dr = xdot_get_sf_dr(ism_band, sf);

    if (dr < 0) {
        return dr;
    }

    return xdot_at_cmd_set_tx_data_rate(&m_xdot_at_cmd_iface, dr);
}

static int xdot_get_tx_max_payload(Xdot_IsmBand ism_band,
        Xdot_SpreadingFactor sf)
{
    int dr = xdot_get_sf_dr(ism_band, sf);

    if (dr < 0) {
        return dr;
    }

    return XDOT_DR_PAYLOAD_MAP[ism_band][dr];
}

static int xdot_get_tx_next(void)
{
    Xdot_AtResp resp = { 0 };
    int ret = 0;

    if ((ret = xdot_at_cmd_get_tx_next(&m_xdot_at_cmd_iface))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    return xdot_buffer_to_int(resp.data, resp.count);
}

static int xdot_get_network_join_status(Xdot_NetworkJoinStatus *status)
{
    Xdot_AtResp resp = { 0 };
    int ret = 0;

    *status = XDOT_NET_JOIN_STATUS_NOT_JOINED;

    if ((ret = xdot_at_cmd_get_net_join_status(&m_xdot_at_cmd_iface))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    if (resp.data[0] == '1') {
        *status = XDOT_NET_JOIN_STATUS_JOINED;
    }
    return PNI_RADIO_RET_OK;
}

static int xdot_set_pre_join_settings(Xdot_Cfg *cfg)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    // set network id
    if ((ret = xdot_at_cmd_set_network_id(&m_xdot_at_cmd_iface,
            cfg->ni_type, cfg->network_app_eui)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 100))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    // set network key
    if ((ret = xdot_at_cmd_set_network_key(&m_xdot_at_cmd_iface,
            cfg->nk_type, cfg->network_app_key)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 100))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    // set link check threshold
    if ((ret = xdot_at_cmd_set_link_check_thrd(&m_xdot_at_cmd_iface,
            XDOT_TX_RETRY_THRD)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 100))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    return PNI_RADIO_RET_OK;
}

static int xdot_set_post_join_settings(Xdot_Cfg *cfg)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    // set TXDR
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_set_tx_data_rate(m_xdot_ism_band, cfg->sf))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    // wait for resp
    if ((ret = xdot_resp_wait(&resp, XDOT_RESP_DELAY_MS))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    return PNI_RADIO_RET_OK;
}

static int xdot_join_if_needed(Xdot_Cfg *cfg)
{
    Xdot_NetworkJoinStatus status = XDOT_NET_JOIN_STATUS_NOT_JOINED;
    int ret;

    // get join status
    ret = xdot_get_network_join_status(&status);
    if (ret != PNI_RADIO_RET_OK) {
        return ret;
    }

    // already joined, return ok
    if (status == XDOT_NET_JOIN_STATUS_JOINED) {
        m_xdot_state.join_retry_count = 0;
        return PNI_RADIO_RET_OK;
    }

    // join needed, apply pre join settings
    if ((ret = xdot_set_pre_join_settings(cfg)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    // do join
    if ((ret = xdot_at_cmd_join(&m_xdot_at_cmd_iface, 1))
            != PNI_RADIO_RET_OK) {
        return ret;
    }
    m_xdot_state.flags |= XDOT_STATE_FLAG_JOIN_WAIT;

    return XDOT_TX_RESP_DELAY_MIN_MS;
}

static int xdot_send_hex(uint8_t *buffer, uint16_t size)
{
    char buff[XDOT_SENDB_MAX_SIZE + 1];
    uint16_t count = 0;
    int i = 0;

    for (i = 0; i < size; i++) {
        count += snprintf(buff + count, sizeof(buff) - count, "%02x", buffer[i]);
    }

    return xdot_at_cmd_sendb(&m_xdot_at_cmd_iface, buff);
}

static int xdot_send_check_tx_resp(void)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    // check for resp
    ret = xdot_get_resp(&resp);

    // if resp recv, clear flag
    if (ret != PNI_RADIO_RET_EBUSY) {
        m_xdot_state.flags &= ~(XDOT_STATE_FLAG_SEND_WAIT);

        // if send ok, return ok
        if (ret == PNI_RADIO_RET_OK) {
            m_xdot_state.tx_retry_count = 0;
            return ret;
        }

        // else ret send delay
        if (m_xdot_state.tx_retry_count++ > XDOT_TX_RETRY_THRD) {
            return m_xdot_state.tx_retry_count * XDOT_TX_RETRY_MULT_MS;
        }
    }

    return ret;
}

static int xdot_send_check_join_resp(Xdot_Cfg *cfg)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    // check for resp
    ret = xdot_get_resp(&resp);

    // resp recv, clear flag
    if (ret != PNI_RADIO_RET_EBUSY) {
        m_xdot_state.flags &= ~(XDOT_STATE_FLAG_JOIN_WAIT);
    }

    switch (ret) {
        case PNI_RADIO_RET_EBUSY:
            return ret;

        case PNI_RADIO_RET_OK:
            // join success, apply settings
            return xdot_set_post_join_settings(cfg);

        case PNI_RADIO_RET_EINVAL:
            // check for retry threshold
            if (m_xdot_state.join_retry_count++ > XDOT_JOIN_RETRY_THRD) {
                return m_xdot_state.join_retry_count * XDOT_JOIN_RETRY_MULT_MS;
            }
            return XDOT_TX_RESP_DELAY_MIN_MS;
    }

    return ret;
}

static int xdot_send_data(void *self, uint8_t *buffer, uint16_t size)
{
    PNI_RadioIface *iface = self;
    Xdot_Cfg *cfg = iface->params;
    int ret;

    XDOT_D("size: %u", size);

    // check waiting for resp
    if (m_xdot_state.flags & XDOT_STATE_FLAG_SEND_WAIT) {
        return xdot_send_check_tx_resp();
    }

    // check waiting for join
    if (m_xdot_state.flags & XDOT_STATE_FLAG_JOIN_WAIT) {
        if ((ret = xdot_send_check_join_resp(cfg)) != PNI_RADIO_RET_OK) {
            return ret;
        }
    }

    // check next tx window
    // if ret > 0 , then is ms to next window, < 0 is error, 0 is ok to send
    ret = xdot_get_tx_next();
    if (ret) {
        /* force next tx to be at least TX_RESP_DELAY_MIN_MS */
        if ((ret > 0) && (ret < XDOT_TX_RESP_DELAY_MIN_MS)) {
            return XDOT_TX_RESP_DELAY_MIN_MS;
        }
        return ret;
    }

    // join if needed
    ret = xdot_join_if_needed(cfg);
    if (ret) {
        return ret;
    }

    // send data
    ret = xdot_send_hex(buffer, size);
    if (ret) {
        return ret;
    }
    m_xdot_state.flags |= XDOT_STATE_FLAG_SEND_WAIT;

    return PNI_RADIO_RET_EBUSY;
}

static int xdot_send(void *self, PNI_ParkingBufferIface *parking_buffer)
{
    PNI_RadioIface *iface = self;
    Xdot_Cfg *cfg = iface->params;
    uint16_t packet_size = sizeof(parking_buffer->buffer.data[0]);
    uint16_t packet_count = parking_buffer->len(parking_buffer);
    uint16_t payload_size = 0;
    uint16_t payload_size_max = 0; /* depends on TXDR */
    uint16_t payload_packet_count = 0;
    uint16_t payload_packet_max = 0; /* depends on payload_size_max */
    PNI_ParkingPacket data[PNI_PARKING_BUFFER_COUNT] = { 0 };
    int index = 0;
    int ret = 0;

    XDOT_D("packet_size: %u, packet_count: %u", packet_size, packet_count);

    /* get max payload size */
    ret = xdot_get_tx_max_payload(m_xdot_ism_band, cfg->sf);
    if (ret < 1) {
        return ret;
    }
    payload_size_max = ret;

    /* maximum number of packets per payload */
    payload_packet_max = (payload_size_max / packet_size);

    /* number of packets to send */
    payload_packet_count = MIN(packet_count, payload_packet_max);

    /* check if already in send */
    if (m_xdot_state.tx_packet_count) {
        /* if caculated packet count is less than tx packet count then
         * something is out of sync. need to figure out how to handle this
         */
        if (payload_packet_count < m_xdot_state.tx_packet_count) {
            XDOT_E("bad packet count");
        }
        payload_packet_count = m_xdot_state.tx_packet_count;
    } else {
        m_xdot_state.tx_packet_count = payload_packet_count;
    }

    /* copy packets from queue into buffer */
    for (index = 0; index < payload_packet_count; index++) {
        parking_buffer->get(parking_buffer, &data[index], index);
    }

    /* size of data to send */
    payload_size = payload_packet_count * packet_size;

    /* if ok, return the number of packets sent */
    ret = xdot_send_data(iface, (uint8_t *)data, payload_size);
    if (ret == PNI_RADIO_RET_OK) {
        /* reset tracked count */
        parking_buffer->adv_out(parking_buffer, payload_packet_count);
        m_xdot_state.tx_packet_count = 0;
    }

    /* ok, error, busy, tx wait (>=1000) */
    return ret;
}

static int xdot_recv(void *self, uint8_t *buffer, uint16_t size)
{
    /* TODO:
     *
     * - Make sure receive buffer (size) is big enough to hold a UART response
     *   (PNI_RADIO_BUFFER_RX_LEN).
     * - Make sure receive buffer is big enough to hold max AT+RECV packet
     *   (242:string), (242:hex -- possible typo in AT reference?)
     */

	XDOT_E("szhang aaa xdot_recv\r\n");
    PNI_RadioIface *iface = self;
    Xdot_Cfg *cfg = iface->params;
    Xdot_AtResp resp = { 0 };
    int ret = PNI_RADIO_RET_OK;  int i;

    if ((ret = xdot_at_cmd_recv(&m_xdot_at_cmd_iface)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    if ((ret = xdot_resp_wait(&resp, XDOT_RESP_DELAY_MS)) != PNI_RADIO_RET_OK) {
        return ret;
    }
                
    /*
     * if echo is enabled and there is no data from the server the respsonse
     * will contain the AT+RECV command, we should ignore it and return ok.
     */
    if (xdot_at_resp_hash_search(resp.hash) == XDOT_AT_RESP_AT_CMD_RECV) {
        return PNI_RADIO_RET_OK;
    }

    if (cfg->rx_type == XDOT_RX_OUT_TYPE_HEX) {

        /* Hex mode returns hex string -- convert into bytes */
        return xdot_hex_to_bytes(resp.data, resp.count, buffer, size);
    } else if (cfg->rx_type == XDOT_RX_OUT_TYPE_RAW) {
        /* Raw mode returns raw bytes */
        memcpy(buffer, &resp.data, resp.count);
        return resp.count;
    } else {
        /* Invalid RX mode */
        return PNI_RADIO_RET_EINVAL;
    }
}

static int xdot_sleep(PNI_RadioSleepMode mode)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    /*
     * This may seem redundant but will be helpful in the future if we
     * use different hardware with different sleep modes.
     */
    switch (mode) {
        case PNI_RADIO_SLEEP_MODE_STANDBY:
            ret = xdot_at_cmd_sleep(&m_xdot_at_cmd_iface,
                    XDOT_SLEEP_MODE_STANDBY);

        case PNI_RADIO_SLEEP_MODE_STOP:
            ret = xdot_at_cmd_sleep(&m_xdot_at_cmd_iface,
                    XDOT_SLEEP_MODE_STOP);
    }

    if (ret != PNI_RADIO_RET_OK) {
        return ret;
    }

    if ((ret = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
        XDOT_E("Failed to enter sleep");
        return ret;
    }

    XDOT_I("going to sleep ...");

    m_xdot_state.flags |= XDOT_STATE_FLAG_IN_SLEEP;

    return PNI_RADIO_RET_OK;
}

static int xdot_reset_mcu(void)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    /* execute reset command */
    if ((ret = xdot_at_cmd_reset(&m_xdot_at_cmd_iface))
            != PNI_RADIO_RET_OK) {
        XDOT_E("Failed to reset MCU");
        return ret;
    }

    HAL_Delay(100);

    /* toggle XDOT_NRST */
    HAL_GPIO_WritePin(XDOT_NRST_GPIO_Port, XDOT_NRST_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(XDOT_NRST_GPIO_Port, XDOT_NRST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    /* wait for device to restart */
    if ((ret = xdot_resp_wait(&resp, 1000)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    return PNI_RADIO_RET_OK;
}

static int xdot_get_device_id(PNI_RadioDeviceId *id)
{
    Xdot_AtResp resp = { 0 };
    int parts[PNI_RADIO_DEVICE_ID_SIZE];
    int i;
    int ret;

    if ((ret = xdot_at_cmd_get_device_id(&m_xdot_at_cmd_iface))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    if (PNI_RADIO_DEVICE_ID_SIZE != sscanf((const char *)resp.data,
            "%X-%X-%X-%X-%X-%X-%X-%X",
            &parts[0], &parts[1], &parts[2], &parts[3],
            &parts[4], &parts[5], &parts[6], &parts[7])) {
        return PNI_RADIO_RET_EINVAL;
    }

    for (i = 0; i < PNI_RADIO_DEVICE_ID_SIZE; i++) {
        id->bytes[i] = (uint8_t)parts[i];
    }

    return PNI_RADIO_RET_OK;
}

static int xdot_detect_ism_band(void)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    if ((ret = xdot_at_cmd_get_freq(&m_xdot_at_cmd_iface))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    ret = xdot_at_resp_hash_search(resp.hash);
    switch (ret) {
        case XDOT_AT_RESP_FREQ_US915:
            m_xdot_ism_band = XDOT_ISM_BAND_US915;
            XDOT_I("Found US915 device");
            break;

        case XDOT_AT_RESP_FREQ_EU868:
            m_xdot_ism_band = XDOT_ISM_BAND_EU868;
            XDOT_I("Found EU868 device");
            break;
    }

    return PNI_RADIO_RET_OK;
}

static int xdot_get_rssi(Xdot_Rssi *rssi)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    if ((ret = xdot_at_cmd_get_rssi(&m_xdot_at_cmd_iface))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    /* TODO: make this not use sscanf */
    if (4 != sscanf((const char *)resp.data, "%f, %f, %f, %f",
            &rssi->last, &rssi->min, &rssi->max, &rssi->avg)) {
        return PNI_RADIO_RET_EINVAL;
    }

    return PNI_RADIO_RET_OK;
}

static int xdot_get_snr(Xdot_Snr *snr)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    if ((ret = xdot_at_cmd_get_snr(&m_xdot_at_cmd_iface))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    /* TODO: make this not use sscanf */
    if (4 != sscanf((const char *)resp.data, "%f, %f, %f, %f",
            &snr->last, &snr->min, &snr->max, &snr->avg)) {
        return PNI_RADIO_RET_EINVAL;
    }

    return PNI_RADIO_RET_OK;
}

static int xdot_ping(void *self, PNI_RadioPong *pong)
{
    Xdot_Rssi rssi = { 0 };
    Xdot_Snr snr = { 0 };
    int ret;

    (void)self;

    /* get RSSI (last, min, max, avg) */
    if ((ret = xdot_get_rssi(&rssi)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    /* get SNR (last, min, max, avg) */
    if ((ret = xdot_get_snr(&snr)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    /* store the last of each */
    pong->rssi = rssi.last;
    pong->snr = snr.last;

    return PNI_RADIO_RET_OK;
}

static int xdot_set_tx_power(void *self, uint8_t dbm)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    if ((ret = xdot_at_cmd_set_tx_power(&m_xdot_at_cmd_iface, dbm))
            != PNI_RADIO_RET_OK) {
        XDOT_E("Set TX power to %u dBm", dbm);
        return ret;
    }

    /* wait for resp */
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    return PNI_RADIO_RET_OK;
}

static int xdot_setup(Xdot_Cfg *cfg)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    // reset MCU
    if ((ret = xdot_reset_mcu()) != PNI_RADIO_RET_OK) {
        XDOT_E("Reset MCU");
        return ret;
    }

    // reset config
    if ((ret = xdot_at_cmd_factory_reset(&m_xdot_at_cmd_iface))
            != PNI_RADIO_RET_OK) {
        XDOT_E("Factory reset");
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    // detect ism band
    if (xdot_detect_ism_band() != PNI_RADIO_RET_OK) {
        return ret;
    }

    // set local echo
    if ((ret = xdot_at_cmd_set_echo_enabled(&m_xdot_at_cmd_iface,
            cfg->echo_enabled)) != PNI_RADIO_RET_OK) {
        XDOT_E("Set local echo to %d", cfg->echo_enabled);
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    // set frequency sub-band (FSB)
    if ((ret = xdot_at_cmd_set_fsb(&m_xdot_at_cmd_iface, cfg->freq_sub_band))
            != PNI_RADIO_RET_OK) {
        XDOT_E("Set FSB to %d", cfg->freq_sub_band);
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    // set public network to enabled
    if ((ret = xdot_at_cmd_set_public_network_enabled(&m_xdot_at_cmd_iface,
            cfg->public_network_enabled)) != PNI_RADIO_RET_OK) {
        XDOT_E("Set public network to %d", cfg->public_network_enabled);
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    // set join mode = OTA
    if ((ret = xdot_at_cmd_set_network_join_mode(&m_xdot_at_cmd_iface,
            cfg->network_join_mode)) != PNI_RADIO_RET_OK) {
        XDOT_E("Set network join mode to %d", cfg->network_join_mode);
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    // set ACKs = 1
    if ((ret = xdot_at_cmd_set_acks_enabled(&m_xdot_at_cmd_iface,
            cfg->acks_enabled)) != PNI_RADIO_RET_OK) {
        XDOT_E("Set acks to %d", cfg->acks_enabled);
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    // set TX power
    if ((ret = xdot_at_cmd_set_tx_power(&m_xdot_at_cmd_iface, cfg->txp))
            != PNI_RADIO_RET_OK) {
        XDOT_E("Set TX power to %u", cfg->txp);
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    // set RX output type
    if ((ret = xdot_at_cmd_set_rx_output(&m_xdot_at_cmd_iface,
            cfg->rx_type)) != PNI_RADIO_RET_OK) {
        XDOT_E("Set RX output to %d", cfg->rx_type);
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    // set wake mode
    if ((ret = xdot_set_wake_mode(XDOT_WAKE_MODE_INTERRUPT))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    // save cfg
    if ((ret = xdot_at_cmd_save_cfg(&m_xdot_at_cmd_iface))
            != PNI_RADIO_RET_OK) {
        XDOT_E("Save config");
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    return PNI_RADIO_RET_OK;
}

static int xdot_test_mode_enable(Xdot_TestParams *params)
{
    /* enable P2P mode */
    Xdot_NetworkJoinMode njm = XDOT_NJM_P2P;
    Xdot_AtResp resp = { 0 };
    int ret;

    /* set join mode = P2P */
    if ((ret = xdot_at_cmd_set_network_join_mode(&m_xdot_at_cmd_iface, njm))
            != PNI_RADIO_RET_OK) {
        XDOT_E("Set network join mode to %d", njm);
        return ret;
    }

    /* wait for resp */
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    /* set P2P TX frequency */
    if ((ret = xdot_at_cmd_set_tx_freq(&m_xdot_at_cmd_iface, params->txf))
            != PNI_RADIO_RET_OK) {
        XDOT_E("Set TX frequency to %s", params->txf);
        return ret;
    }

    /* wait for resp */
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    /* set P2P TX power */
    if ((ret = xdot_at_cmd_set_tx_power(&m_xdot_at_cmd_iface, params->txp))
            != PNI_RADIO_RET_OK) {
        XDOT_E("Set TX power to %u", params->txp);
        return ret;
    }

    /* wait for resp */
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    /* enable send on interval */
    if ((ret = xdot_at_cmd_send_on_interval(&m_xdot_at_cmd_iface,
            params->interval_ms, params->msg)) != PNI_RADIO_RET_OK) {
        XDOT_E("Enable send on interval");
        return ret;
    }

    return PNI_RADIO_RET_OK;
}

static int xdot_test_mode_disable(Xdot_Cfg *cfg)
{
    const char esc = '+';
    Xdot_AtResp resp = { 0 };
    int ret;

    /* set resp wait flag */
    m_xdot_state.flags |= XDOT_STATE_FLAG_RESP_WAIT;

    /* manually send +++ escape chars */
    HAL_UART_Transmit(m_xdot_huart, (uint8_t *)&esc, 1, PNI_RADIO_UART_TIMEOUT);
    Clock_Wait(2);
    HAL_UART_Transmit(m_xdot_huart, (uint8_t *)&esc, 1, PNI_RADIO_UART_TIMEOUT);
    Clock_Wait(2);
    HAL_UART_Transmit(m_xdot_huart, (uint8_t *)&esc, 1, PNI_RADIO_UART_TIMEOUT);
    Clock_Wait(2);

    /* wait for resp */
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 5000)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    /* reset and setup xDot */
    return xdot_setup(cfg);
}

static int xdot_set_test_mode_enabled(void *self, uint8_t enabled, void *params)
{
    PNI_RadioIface *iface = self;
    Xdot_Cfg *cfg = iface->params;

    if (enabled) {
        return xdot_test_mode_enable(params);
    }

    return xdot_test_mode_disable(cfg);
}

static int xdot_configure(void *self, void *params)
{
    PNI_RadioIface *iface = self;
    Xdot_Cfg *cfg = params;

    iface->params = params;

    return xdot_setup(cfg);
}

int pni_radio_xdot_init(PNI_RadioIface *iface, UART_HandleTypeDef *huart)
{
    m_xdot_huart = huart;
    iface->send = &xdot_send;
    iface->recv = &xdot_recv;
    iface->sleep = &xdot_sleep;
    iface->reset = &xdot_reset_mcu;
    iface->get_device_id = &xdot_get_device_id;
    iface->ping = &xdot_ping;
    iface->set_test_mode_enabled = &xdot_set_test_mode_enabled;
    iface->configure = &xdot_configure;
    iface->set_tx_power = &xdot_set_tx_power;
    // init xdot_at_cmd_iface
    m_xdot_at_cmd_iface.buffer = m_xdot_buff_tx;
    m_xdot_at_cmd_iface.size = sizeof(m_xdot_buff_tx);
    m_xdot_at_cmd_iface.exec = &xdot_at_cmd_exec;

    xdot_at_resp_hash_init();

    /* TODO: error handling for HAL functions */
    HAL_GPIO_WritePin(XDOT_NRST_GPIO_Port, XDOT_NRST_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(XDOT_NRST_GPIO_Port, XDOT_NRST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    __HAL_UART_FLUSH_DRREGISTER(huart);
    HAL_UART_Receive_DMA(huart, (uint8_t *)&m_xdot_buff_rx,
            sizeof(m_xdot_buff_rx));

    return PNI_RADIO_RET_OK;
}
