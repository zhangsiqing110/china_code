/**
* @file			SENtralA2.h
*
* @brief		SENtralA2 header
*
* @date			12/08/2016
* @copyright    (C) 2016 PNI Corp
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

#ifndef SENTRAL_A2_H
#define SENTRAL_A2_H

#include <stdint.h>

#include "SENtralA2_types.h"
#include "stm32l0xx_hal.h"
#include "parking_buffer.h"

#define SENA2_ENABLE_DEBUG_PRINT (1)

#define SENA2_I(fmt, ...) PNI_PRINTF("[INFO][SENA2] " fmt, ##__VA_ARGS__)
#define SENA2_E(fmt, ...) PNI_PRINTF("[ERROR][SENA2] " fmt, ##__VA_ARGS__)

#if SENA2_ENABLE_DEBUG_PRINT
#define SENA2_D(fmt, ...) PNI_PRINTF("[DBG][SENA2] %s() " fmt, __func__, ##__VA_ARGS__)
#else /* SENA2_ENABLE_DEBUG_PRINT */
#define SENA2_D(fmt, ...)
#endif /* SENA2_ENABLE_DEBUG_PRINT */

#define SENSOR_DATA_BUFFER_MAX                  32U
#define PARK_SNS_PAYLOAD_SIZE                   12

extern volatile uint8_t A2Interrupt;

#pragma pack(push, 1)
typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} ParkingRelease;
#pragma pack(pop)

HAL_StatusTypeDef SentralWrite(const uint8_t RegAddress, const uint8_t data);
HAL_StatusTypeDef SentralRead(uint8_t RegAddress,uint8_t *data);

void displayA2StatusRegisters();
void displayDeviceIdRegisters();

int SENtralA2_i2c_read(uint8_t reg, uint8_t *data, uint16_t size);
int SENtralA2_i2c_read_value(uint8_t reg, uint8_t *value);
int SENtralA2_i2c_write(uint8_t reg, uint8_t *data, uint16_t size);
int SENtralA2_i2c_write_value(uint8_t reg, uint8_t value);

uint32_t SENtralA2_param_read(uint8_t *values, uint8_t page, ParamInfo *paramList, uint8_t numParams);
uint32_t SENtralA2_param_write(uint8_t *values, uint8_t page, ParamInfo *paramList, uint8_t numParams);


uint32_t SENtralA2_set_sensor_rate(uint8_t sensorId, uint16_t rate);

int SENtralA2_read_fifo(uint8_t *buffer);
uint32_t SENtralA2_get_fifo(uint8_t *buffer, uint16_t numBytes);
int SENtralA2_getBytesRemaining(uint16_t *bytes);
int SENtralA2_init(SENtralA2Iface *iface);
int SENtralA2_parse_fifo(SENtralA2Iface *iface, uint8_t *buffer, uint16_t size,
    SENtralA2_SensorPacket *sns);

void SENtralA2_GPIO_EXTI_Callback();

/**
 * @brief  function to send data frame through Lora(XDot) interface to Cloud
 * @param  uint8_t *  data
 *        uint16_t  data size
 *        uint32_t  retry - number of retry to check for response, each retry the check period will double. starting with 10ms
 *        int32_t * loraRsp - (output) return lora XDot response
 *
 * @retval Return indicate data send is confirmed (0/1 = DISABLE(NO)/ENABLE(YES))
 */
FunctionalState SENtralA2_SendFrame2Lora(uint8_t * data, uint16_t dataSize, uint8_t retry, int32_t * loraRsp);

#endif  // SENTRAL_A2_H
