#ifndef PNI_RADIO_H
#define PNI_RADIO_H

#include <stdint.h>
#include "parking_buffer.h"

#define PNI_RADIO_UART_TIMEOUT (5000)
#define PNI_RADIO_BUFFER_TX_LEN (512)
#define PNI_RADIO_BUFFER_RX_LEN (256) /* *MUST* be power of 2 */
#define PNI_RADIO_RESP_DELAY_MS (10)

typedef enum pni_radio_ret {
    PNI_RADIO_RET_EINVAL = -22,
    PNI_RADIO_RET_EBUSY =  -16,
    PNI_RADIO_RET_EIO =     -5,
    PNI_RADIO_RET_OK =       0,
} PNI_RadioRet;

typedef enum pni_radio_sleep_mode {
    PNI_RADIO_SLEEP_MODE_STANDBY = 0,
    PNI_RADIO_SLEEP_MODE_STOP = 1,
} PNI_RadioSleepMode;

#define PNI_RADIO_DEVICE_ID_SIZE (8)
typedef struct pni_radio_device_id {
    uint8_t bytes[PNI_RADIO_DEVICE_ID_SIZE];
} PNI_RadioDeviceId;

typedef struct pni_radio_pong {
    float rssi;
    float snr;
} PNI_RadioPong;

typedef struct pni_radio_iface {

    /*
     * Send data to server
     *
     * The implementation of this function may break the buffer up into
     * multiple transmissions. To avoid breaking up packets across
     * transmissions, the number of packets in the in buffer is passed in so
     * the packet size can be calculated to fit into transmissions. The count
     * pointer will be decremented for each success so that data is not
     * duplicated if a failure retry is needed.
     *
     * @param self pointer to this interface instance
     * @param parking_buffer pointer to circular buffer interface
     * @return zero on success
     * @return greater than zero millseconds before next TX window
     * @return (-16) EBUSY on waiting for AT response
     * @return (-22) EINVAL on AT response returned ERROR
     */
    int (*send)(void *self, PNI_ParkingBufferIface *parking_buffer);

    /*
     * Receive data from server
     *
     * @param self pointer to this interface instance
     * @param buffer buffer to write received data into
     * @param size size of buffer
     * @return negative on error
     * @return zero on sucess and no data received
     * @return number of bytes received and stored into buffer
     */
    int (*recv)(void *self, uint8_t *buffer, uint16_t size);

    /*
     * Put xDot into sleep mode
     *
     * @param mode sleep mode to use
     * @return zero on success
     * @return negative on error
     */
    int (*sleep)(PNI_RadioSleepMode mode);

    /*
     * Reset the xDot's MCU
     *
     * @return zero on success
     * @return negative on error
     */
    int (*reset)(void);

    /*
     * Get the xDot's unique, 8-byte LoRa node id
     *
     * @param id pointer to store id into
     * @return zero on succes
     * @return negative on error
     */
    int (*get_device_id)(PNI_RadioDeviceId *id);

    int (*ping)(void *self, PNI_RadioPong *pong);

    /*
     * Toggle test mode
     *
     * @param self pointer to this interface instance
     * @param enabled 0 = disabled, 1 = enabled
     * @param params test params
     * @return zero on succes
     * @return negative on error
     */
    int (*set_test_mode_enabled)(void *self, uint8_t enabled, void *params);

    /*
     * Configure radio
     *
     * @param self pointer to this interface instance
     * @param param pointer to radio configuration parameters
     * @return zero on succes
     * @return negative on error
     */
    int (*configure)(void *self, void *params);

    /*
     * Configure transmit power in dBm before antennae gain
     *
     * @param self pointer to this interface instance
     * @param dbm tx power in dBm
     * @return zero on succes
     * @return negative on error
     */
    int (*set_tx_power)(void *self, uint8_t dbm);

    /*
     * Parameters to pass to the implementation
     */
    void *params;
} PNI_RadioIface;

#endif /* PNI_RADIO_H */
