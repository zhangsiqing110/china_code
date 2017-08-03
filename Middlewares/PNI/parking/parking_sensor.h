/**
* @file         parking_sensor.h
*
* @brief        PNI Parking
*
* @date         03/09/2017
* @copyright    (C) 2017 PNI Corp
*
*               THIS SOFTWARE IS PROVIDED BY PNI SENSOR CORPORATION "AS IS" AND
*               ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
*               TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
*               PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL PNI SENSOR
*               CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*               SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
*               NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*               LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*               HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*               CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
*               OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
*               EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*               DISCLOSURE TO THIRD PARTIES OR REPRODUCTION IN ANY FORM
*               WHATSOEVER, WITHOUT PRIOR WRITTEN CONSENT, IS STRICTLY
*               FORBIDDEN.
*
*/

#ifndef PNI_PARKING_SENSOR_H
#define PNI_PARKING_SENSOR_H

#include <stdint.h>
#include "parking.h"

typedef struct pni_parking_sensor_iface {
    /*
     * Enable/Disable the Car Detector sensor
     *
     * @param self pointer to iface impl
     * @param enabled 0 = disabled, 1 = enabled
     * @return zero on success
     * @return negative on error
     */
    int (*set_car_detector_enabled)(void *self, int enabled);

    /*
     * Enable/Disable the Car Detector Data sensor
     *
     * @param self pointer to iface impl
     * @param enabled 0 = disabled, 1 = enabled
     * @return zero on success
     * @return negative on error
     */
    int (*set_car_detector_data_enabled)(void *self, int enabled);

    /*
     * Enable/Disable the Car Detector intermediate state reporting
     *
     * @param self pointer to iface impl
     * @param enabled 0 = disabled, 1 = enabled
     * @return zero on success
     * @return negative on error
     */
    int (*set_inter_state_enabled)(void *self, int enabled);

    /*
     * Force Car Detector state to "occupied"
     *
     * @param self pointer to iface impl
     * @return zero on success
     * @return negative on error
     */
    int (*force_occupied)(void *self);

    /*
     * Force Car Detector state to "vacant"
     *
     * @param self pointer to iface impl
     * @return zero on success
     * @return negative on error
     */
    int (*force_vacant)(void *self);

    /*
     * Recalibrate the parking sensor
     *
     * @param self pointer to iface impl
     * @return zero on success
     * @return negative on error
     */
    int (*recalibrate)(void *self);

    /*
     * Initiate a self-test of the parking sensor
     *
     * @param self pointer to iface impl
     * @return zero on success
     * @return negative on error
     */
    int (*self_test)(void *self);

    /*
     * Get the parking sensor firmware version
     *
     * @param self pointer to iface impl
     * @param version pointer to write version into
     * @return zero on success
     * @return negative on error
     */
    int (*get_version)(void *self, PNI_ParkingVersion *version);

    int (*set_hw_alarm_mode_enabled)(void *self, int enabled);

    int (*set_temperature)(void *self, int8_t temperature);

    int (*is_maint_triggered)(void *self);

    /* Check manufacture mode Flag(pattern E) is enabled or not
     * @param void * self - Parking sensor interface
     * @retval int8_t Return value  ( -1/0/1 == Error/No/Yes(triggered) )
     */
    int (*is_mfg_triggered)(void *self);

    /*
     * Check shipping mode Flag is enabled or not
     *
     * @param @param self pointer to iface impl
     * @return -1/0/1 == Error/Not Enabled/Enabled
     */
    int (*is_shipping_mode_enabled)(void *self);

    int (*set_shipping_mode_enabled)(void *self, uint8_t enabled);
    /*
     * Set parking sensor parameter
     *
     * @param self pointer to iface impl
     * @param page parameter page
     * @param num parameter number
     * @param data pointer to data to read from and write into sensor
     * @param size size of the data to read
     * @return zero on success
     * @return negative on error
     */
    int (*set_parameter)(void *self, uint8_t page, uint8_t num,
            uint8_t *data, uint8_t size);

    /*
     * Get parking sensor parameter
     *
     * @param self pointer to iface impl
     * @param page parameter page
     * @param num parameter number
     * @param data pointer to data to write into from sensor
     * @param size size of the data to write
     * @return zero on success
     * @return negative on error
     */
    int (*get_parameter)(void *self, uint8_t page, uint8_t num,
            uint8_t *data, uint8_t size);

    void *params;
} PNI_ParkingSensorIface;

#endif
