/**
* @file			SENtralA2.h
*
* @brief		SENtralA2
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

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "SENtralA2.h"
#include "stm32l0xx_hal.h"
#include "pni_config.h"
#include "wwdg.h"
#include "ble_clock.h"
#include "string.h"
#include "parking_sensor.h"
#if ENABLE_XDOT_RADIO
#include "xdot.h"
#endif /* ENABLE_XDOT_RADIO */

//#define ENABLE_DBG_MSG
#ifdef ENABLE_DBG_MSG
#define SENTRAL_A2_DBG  1
#else
#define SENTRAL_A2_DBG  0
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t A2Interrupt = 0;

/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

int SENtralA2_i2c_read(uint8_t reg, uint8_t *data, uint16_t size)
{
  if (HAL_I2C_Mem_Read(&SENTRAL_A2_I2C, SENTRAL_A2_ADDRESS, reg,
      I2C_MEMADD_SIZE_8BIT, data, size, SENTRAL_A2_I2C_TIMEOUT)
      != HAL_OK) {
    return SENA2_RET_ERROR;
  }
  return SENA2_RET_OK;
}

int SENtralA2_i2c_read_value(uint8_t reg, uint8_t *value)
{
  return SENtralA2_i2c_read(reg, value, sizeof(*value));
}

int SENtralA2_i2c_write(uint8_t reg, uint8_t *data, uint16_t size)
{
  if (HAL_I2C_Mem_Write(&SENTRAL_A2_I2C, SENTRAL_A2_ADDRESS, reg,
      I2C_MEMADD_SIZE_8BIT, data, size, SENTRAL_A2_I2C_TIMEOUT)
      != HAL_OK) {
    return SENA2_RET_ERROR;
  }
  return SENA2_RET_OK;
}

int SENtralA2_i2c_write_value(uint8_t reg, uint8_t value)
{
  return SENtralA2_i2c_write(reg, &value, sizeof(value));
}

uint8_t SENtralA2_is_ready()
{
  uint16_t a2I2cAddr = SENTRAL_A2_ADDRESS;
  // check if Sentral is Ready for communication
  HAL_StatusTypeDef x = HAL_I2C_IsDeviceReady(&SENTRAL_A2_I2C,(uint16_t)a2I2cAddr,1,1000);

  while(x != HAL_OK)
  {
    if(x == HAL_ERROR)
    {
      SENA2_E("I2C Device(%d) unable to communicate\r\n", a2I2cAddr);
      Error_Handler();
      return PNI_NO;
    }
    if(x == HAL_BUSY)
    {
      //Clock_Wait(200);
      SENA2_E("I2C device(%d) still busy - Try again\r\n", a2I2cAddr);
      // TODO: handle Sentral reset failure
    }
    x=HAL_I2C_IsDeviceReady(&SENTRAL_A2_I2C,(uint16_t)(0x50>>1),1,1000);
  }

  SENA2_I("I2C started...communicated with device(%d)\r\n", a2I2cAddr);
  return PNI_YES;
}

char* A2strBits(void const * const ptr, uint8_t numBytes, char* str)
{
	uint8_t *bytes = (uint8_t *)ptr;
	uint8_t i, j;
	for (i = 0; i < numBytes; i++)
	{
		for (j = 0; j < 8; j++)
		{
			str[i * 8 + (7 - j)] = bytes[(numBytes - 1) - i] & (1 << j) ? '1' : '0';
		}
	}
	str[numBytes * 8] = '\0';
	return str;
}

void displayDeviceIdRegisters()
{
	uint8_t buf[2];
	char str[17];
	PNI_PRINTF("\r\n------------ Displaying ID Registers -----------\r\n");
	SENtralA2_i2c_read(PRODUCT_ID_REG, buf, 2);
	PNI_PRINTF("Product ID:       % 5u, %s\r\n", buf[0], A2strBits(&buf[0], sizeof(buf[0]), str));
	PNI_PRINTF("Revision ID:      % 5u, %s\r\n", buf[1], A2strBits(&buf[1], sizeof(buf[0]), str));
}

void displayA2StatusRegisters()
{
	uint8_t buf[4];
	char str[17];
	PNI_PRINTF("\r\n------------ Displaying Status Registers -----------\r\n");
	SENtralA2_i2c_read(CHIP_CONTROL_REG, buf, 4);
	PNI_PRINTF("Chip Control[0x34]:      % 5u, %s\r\n", buf[0], A2strBits(&buf[0], sizeof(buf[0]), str));
	PNI_PRINTF("Host Status[0x35]:       % 5u, %s\r\n", buf[1], A2strBits(&buf[1], sizeof(buf[0]), str));
	PNI_PRINTF("Interrupt Status[0x36]:  % 5u, %s\r\n", buf[2], A2strBits(&buf[2], sizeof(buf[0]), str));
	PNI_PRINTF("Chip Status[0x37]:       % 5u, %s\r\n", buf[3], A2strBits(&buf[3], sizeof(buf[0]), str));
	SENtralA2_i2c_read(ERR_REG, buf, 4);
	PNI_PRINTF("Err Register[0x50]:      % 5u, %s\r\n", buf[0], A2strBits(&buf[0], sizeof(buf[0]), str));
	PNI_PRINTF("Interrupt State[0x51]:   % 5u, %s\r\n", buf[1], A2strBits(&buf[1], sizeof(buf[0]), str));
	PNI_PRINTF("Debug Value[0x52]:       % 5u, %s\r\n", buf[2], A2strBits(&buf[2], sizeof(buf[0]), str));
	PNI_PRINTF("Debug State[0x53]:       % 5u, %s\r\n", buf[3], A2strBits(&buf[3], sizeof(buf[0]), str));
	SENtralA2_i2c_read(BYTES_REMANING_REG, buf, 2);
	uint16_t *v = (uint16_t *)&buf;
	PNI_PRINTF("Bytes Remaining[0x38]:   % 5u, %s\r\n\n", v[0], A2strBits(&v[0], sizeof(v[0]), str));
}

/**
  * @brief GPIO EXTI callback
  * @param None
  * @retval None
  */
void SENtralA2_GPIO_EXTI_Callback()
{
  //PNI_PRINTF("   A2-->DRDY, A2Interrupt = %u\n", A2Interrupt);
  //PNI_PRINTF("A");
  //A2Interrupt = 1;
  A2Interrupt++;

  /* Clear Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}

/*******************************************************************************
* Function Name  : SentralWrite
* Description    : Writes a byte of data to sentral
* Input          : - Register address to write to
*                  - Data to write.
* Return         : status of write operation
*******************************************************************************/

HAL_StatusTypeDef SentralWrite(const uint8_t RegAddress, const uint8_t data){


  uint8_t SMB_DATA_OUT[3];

  //SMB_DATA_OUT[0] = ;	// Sentral shifted Slave Address
  SMB_DATA_OUT[0] = RegAddress;	// Register number
  SMB_DATA_OUT[1] = data;	// Data to write

  return HAL_I2C_Master_Transmit(&SENTRAL_A2_I2C, 0x50, SMB_DATA_OUT, 2, 1000);

}
/*******************************************************************************
* Function Name  : SentralRead
* Description    : Reads a byte of data from sentral's address
* Input          : - Sentral's register address to read from
* Return         : status of read operation
*******************************************************************************/

HAL_StatusTypeDef SentralRead(uint8_t RegAddress,uint8_t *data)
{
  data[0] = 0;
  HAL_StatusTypeDef ret  = HAL_OK;

  ret|= HAL_I2C_Master_Transmit(&SENTRAL_A2_I2C, 0x50, &RegAddress, 1, 1000);
  ret|= HAL_I2C_Master_Receive(&SENTRAL_A2_I2C, 0x50, data, 1, 1000);

  return ret;
}

uint32_t SENtralA2_param_read(uint8_t *values, uint8_t page, ParamInfo *paramList, uint8_t numParams)
{
	uint8_t i, paramAck, pageSelectValue;
	uint16_t valIndex = 0, retry;

	for (i = 0; i < numParams; i++)
	{
		retry = 0;
		pageSelectValue = page | (paramList[i].size << 4);
		SENtralA2_i2c_write_value(PARAM_PAGE_SELECT_REG, pageSelectValue);
		SENtralA2_i2c_write_value(PARAM_REQUEST_REG, paramList[i].paramNo);
		do
		{
			//SENtralA2_delay_ms(1); // remove the delay, unless there is problem with param read
			SENtralA2_i2c_read(PARAM_ACK_REG, &paramAck, 1);

      // retry need to be at least 400 for Tethered Board
			if (paramAck == 0x80 || retry > 400)
			{
				SENtralA2_i2c_write_value(PARAM_REQUEST_REG, 0);
				SENtralA2_i2c_write_value(PARAM_PAGE_SELECT_REG, 0);
				//SENtralA2_unlock_mutex(paramIoMutex);
				return 0;
			}
			retry++;
		} while (paramAck != paramList[i].paramNo);
		SENtralA2_i2c_read(PARAM_SAVE_REG, &values[valIndex], paramList[i].size);
		valIndex += paramList[i].size;
	}
	SENtralA2_i2c_write_value(PARAM_REQUEST_REG, 0);
	SENtralA2_i2c_write_value(PARAM_PAGE_SELECT_REG, 0);

	return 1;
}

uint32_t SENtralA2_param_write(uint8_t *values, uint8_t page, ParamInfo *paramList, uint8_t numParams)
{
	uint8_t i, paramAck, paramNum, pageSelectValue;
	uint16_t valIndex = 0, retry;
	for (i = 0; i < numParams; i++)
	{
		retry = 0;
		pageSelectValue = page | (paramList[i].size << 4);
		SENtralA2_i2c_write_value(PARAM_PAGE_SELECT_REG, pageSelectValue);

		SENtralA2_i2c_write(PARAM_LOAD_REG, &values[valIndex], (uint16_t)paramList[i].size);
    //Clock_Wait(10);   // don't need the delay unless problem writing params

		paramNum = paramList[i].paramNo | 0x80;
		SENtralA2_i2c_write_value(PARAM_REQUEST_REG, paramNum);
    //Clock_Wait(10);   // don't need the delay unless problem writing params
		do
		{
			//SENtralA2_delay_ms(8); //delay not needed unless problem writing params
      SENtralA2_i2c_read_value(PARAM_ACK_REG, &paramAck);

      // retry need to be at least 400 for Tethered Board
			if (paramAck == 0x80 || retry > 400)
			{
        SENA2_E("param write failed - %d\r\n", paramList[i].paramNo);
				SENtralA2_i2c_write_value(PARAM_REQUEST_REG, 0);
				SENtralA2_i2c_write_value(PARAM_PAGE_SELECT_REG, 0);
				return 0;
			}
			retry++;
		} while (paramAck != paramNum);

		valIndex += paramList[i].size;
	}

	SENtralA2_i2c_write_value(PARAM_REQUEST_REG, 0);
	SENtralA2_i2c_write_value(PARAM_PAGE_SELECT_REG, 0);
	return 1;
}

int SENtralA2_getBytesRemaining(uint16_t *bytes)
{
  int result = 0;

  result = SENtralA2_i2c_read(BYTES_REMANING_REG, (uint8_t *)bytes,
      sizeof(*bytes));

  if (result != SENA2_RET_OK) {
    SENA2_E("problem reading BYTES REMAINING register, error = %u!!\n", result);
  }

  return result;
}

static void SENtralA2_handle_meta_event(SENtralA2Iface *iface,
    SENtralA2_MetaEvent *meta_event)
{
  SENA2_I("Meta Event: id: 0x%02X, byte1: 0x%02X, byte2: 0x%02X\r\n",
      meta_event->id, meta_event->bytes[0], meta_event->bytes[1]);

  /* no callback iface set */
  if (iface->meta_event_cb == NULL) {
    return;
  }

  /* exec callbacks */
  switch (meta_event->id) {

    case META_EVENT_FLUSH_COMPLETE:
      if (iface->meta_event_cb->on_flush_complete != NULL) {
        iface->meta_event_cb->on_flush_complete(iface->meta_event_cb,
            meta_event->bytes[0]);
      }
      break;

    case META_EVENT_SAMPLE_RATE_CHANGED:
      if (iface->meta_event_cb->on_sample_rate_changed != NULL) {
        iface->meta_event_cb->on_sample_rate_changed(iface->meta_event_cb,
            meta_event->bytes[0]);
      }
      break;

    case META_EVENT_POWER_MODE_CHANGED:
      if (iface->meta_event_cb->on_power_mode_changed != NULL) {
        iface->meta_event_cb->on_power_mode_changed(
            iface->meta_event_cb, meta_event->bytes[0], meta_event->bytes[1]);
      }
      break;

    case META_EVENT_ERROR:
      if (iface->meta_event_cb->on_error != NULL) {
        iface->meta_event_cb->on_error(
            iface->meta_event_cb, meta_event->bytes[0], meta_event->bytes[1]);
      }
      break;

    case META_EVENT_SENSOR_EVENT:
      if (iface->meta_event_cb->on_sensor_error != NULL) {
        iface->meta_event_cb->on_sensor_error(
            iface->meta_event_cb, meta_event->bytes[0], meta_event->bytes[1]);
      }
      break;

    case META_EVENT_FIFO_OVERFLOW:
      if (iface->meta_event_cb->on_fifo_overflow != NULL) {
        iface->meta_event_cb->on_fifo_overflow(iface->meta_event_cb,
            meta_event->value);
      }
      break;

    case META_EVENT_DYNAMIC_RANGE_CHANGED:
      if (iface->meta_event_cb->on_dynamic_range_changed != NULL) {
        iface->meta_event_cb->on_dynamic_range_changed(iface->meta_event_cb,
            meta_event->bytes[0]);
      }
      break;

    case META_EVENT_FIFO_WATERMARK:
      if (iface->meta_event_cb->on_fifo_watermark != NULL) {
        iface->meta_event_cb->on_fifo_watermark(iface->meta_event_cb,
            meta_event->value);
      }
      break;

    case META_EVENT_SELF_TEST_RESULT:
      if (iface->meta_event_cb->on_self_test_results != NULL) {
        iface->meta_event_cb->on_self_test_results(
            iface->meta_event_cb, meta_event->bytes[0], meta_event->bytes[1]);
      }
      break;

    case META_EVENT_INITIALIZED:
      if (iface->meta_event_cb->on_initialized != NULL) {
        iface->meta_event_cb->on_initialized(iface->meta_event_cb,
            meta_event->value);
      }
      break;
  }
}

static void SENtralA2_handle_stime(SENtralA2Iface *iface, uint8_t sid,
    uint16_t stime)
{
  switch (sid) {

    case SENSOR_TYPE_TIMESTAMP:
      iface->stime_nw = (iface->stime_nw & 0xFFFF0000) | stime;
      break;

    case SENSOR_TYPE_TIMESTAMP_WAKE:
      iface->stime_wk = (iface->stime_nw & 0xFFFF0000) | stime;
      break;

    case SENSOR_TYPE_TIMESTAMP_OVERFLOW:
      iface->stime_nw = ((uint32_t)stime << 16);
      break;

    case SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE:
      iface->stime_wk = ((uint32_t)stime << 16);
      break;
  }
}

int SENtralA2_parse_fifo(SENtralA2Iface *iface, uint8_t *buffer, uint16_t size,
    SENtralA2_SensorPacket *sns)
{
  SENtralA2_FifoData *data = (SENtralA2_FifoData *)&buffer[1];

  memset(sns, 0, sizeof(*sns));

  /* set sensor id */
  sns->sid = buffer[0];

  /* do any special handling of FIFO packets */
  switch (sns->sid) {
    case SENSOR_TYPE_TIMESTAMP:
    case SENSOR_TYPE_TIMESTAMP_WAKE:
    case SENSOR_TYPE_TIMESTAMP_OVERFLOW:
    case SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE:
      SENtralA2_handle_stime(iface, sns->sid, data->stime);
      break;

    case SENSOR_TYPE_META:
    case SENSOR_TYPE_META_WAKE:
      SENtralA2_handle_meta_event(iface, &data->meta_event);
      break;
  }

  /* set timestamp */
  if ((sns->sid < SENSOR_TYPE_ACCELEROMETER_WAKE)
      || (sns->sid == SENSOR_TYPE_DEBUG)
      || (sns->sid == SENSOR_TYPE_TIMESTAMP)
      || (sns->sid == SENSOR_TYPE_TIMESTAMP_OVERFLOW)
      || (sns->sid == SENSOR_TYPE_META)
      || (sns->sid == SENSOR_TYPE_RAW_GYRO)
      || (sns->sid == SENSOR_TYPE_RAW_MAG)
      || (sns->sid == SENSOR_TYPE_RAW_ACCEL)) {
    sns->timestamp = iface->stime_nw;
  } else {
    sns->timestamp = iface->stime_wk;
  }

  /* copy data */
  if (iface->event_sizes[sns->sid] > 0) {
    /* set data size, not including sensor id */
    sns->count = iface->event_sizes[sns->sid] - 1;
    memcpy(sns->data.bytes, data, sns->count);
  }

  return iface->event_sizes[sns->sid];
}

int SENtralA2_read_fifo(uint8_t *buffer)
{
  // Check number of bytes available
  uint8_t bytesAvailable = 0, bytesRead = 0;
  uint16_t bytesToRead = 0;

  SENtralA2_i2c_read(BYTES_REMANING_REG, (uint8_t *)&bytesAvailable,
      sizeof(bytesAvailable));

  while (bytesAvailable > 0) {
    // Break on 50 byte fifo register block
    bytesToRead = ((bytesRead % 50) + I2C_MAX_READ) > 50
        ? 50 - (bytesRead % 50)
        : I2C_MAX_READ;

    // Make sure we don't read more than is available in the fifo
    bytesToRead = MIN(bytesAvailable, bytesToRead);
    if (SENtralA2_i2c_read(bytesRead % 50, &buffer[bytesRead], bytesToRead)
        != SENA2_RET_OK) {
      SENA2_E("error reading FIFO\r\n");
      return SENA2_RET_ERROR;
    }
    bytesAvailable -= bytesToRead;
    bytesRead += bytesToRead;
  }
  return bytesRead;
}

uint32_t SENtralA2_set_sensor_rate(uint8_t sensorId, uint16_t rate)
{
  uint8_t paramPage = PARAM_PAGE_SENSOR_CONF;
  ParamInfo param[] = { sensorId, sizeof(rate) };
  return SENtralA2_param_write((uint8_t *)&rate, paramPage, param, 1);
}

int SENtralA2_self_test(void *self)
{
  uint8_t val = 0;

  //Enable algo standby
  SENtralA2_i2c_read_value(HOST_INTERFACE_CTRL_REG, &val);
  val |= HOST_IFACE_CTRL_FLAG_ALGORITHM_STANDBY;
  SENtralA2_i2c_write_value(HOST_INTERFACE_CTRL_REG, val);

  // Wait for algo standby
  val = 0;
  do {
    HAL_Delay(10);
    SENtralA2_i2c_read_value(HOST_STATUS_REG, &val);
  } while (!(val & HOST_STATUS_ALGORITHM_STANDBY));

  // Enable self test and disable algo standby
  SENtralA2_i2c_read_value(HOST_INTERFACE_CTRL_REG, &val);
  val |= (HOST_IFACE_CTRL_FLAG_REQ_SENSOR_SELF_TEST);
  val &= ~(HOST_IFACE_CTRL_FLAG_ALGORITHM_STANDBY);
  return SENtralA2_i2c_write_value(HOST_INTERFACE_CTRL_REG, val);
}

static int SENtralA2_set_parameter(void *self, uint8_t page, uint8_t num,
    uint8_t *data, uint8_t size)
{
  ParamInfo info[] = { num, size };

  return SENtralA2_param_write(data, page, info, 1);
}

static int SENtralA2_get_parameter(void *self, uint8_t page, uint8_t num,
    uint8_t *data, uint8_t size)
{
  ParamInfo info[] = { num, size };

  return SENtralA2_param_read(data, page, info, 1);
}

static int SENtralA2_set_sns_rate(void *self, uint8_t sid, uint16_t rate)
{
  return SENtralA2_set_sensor_rate(sid, rate);
}

static int SENtralA2_set_meta_event_ctrl(void *self, uint64_t nonwake,
    uint64_t wake)
{
  ParamInfo info[] = {
    { .paramNo = PARAM_META_EVENT_CONTROL, .size = sizeof(nonwake) },
    { .paramNo = PARAM_WAKE_META_EVENT_CONTROL, .size = sizeof(wake) },
  };
  uint64_t data[] = { nonwake, wake };
  uint8_t val = 0;
  int ret = SENA2_RET_OK;

  //Enable algo standby
  SENtralA2_i2c_read_value(HOST_INTERFACE_CTRL_REG, &val);
  val |= HOST_IFACE_CTRL_FLAG_ALGORITHM_STANDBY;
  SENtralA2_i2c_write_value(HOST_INTERFACE_CTRL_REG, val);

  // Wait for algo standby
  val = 0;
  do {
    HAL_Delay(10);
    SENtralA2_i2c_read_value(HOST_STATUS_REG, &val);
  } while (!(val & HOST_STATUS_ALGORITHM_STANDBY));

  // set meta event ctrl params
  SENtralA2_param_write((uint8_t *)data, PARAM_PAGE_SYSTEM, info, 2);
  // ret = SENtralA2_set_parameter(self, PARAM_PAGE_SYSTEM, PARAM_META_EVENT_CONTROL, (uint8_t *)&nonwake, sizeof(nonwake));
  if (ret != SENA2_RET_OK) {
    return ret;
  }

  // disable algo standby
  SENtralA2_i2c_read_value(HOST_INTERFACE_CTRL_REG, &val);
  val &= ~(HOST_IFACE_CTRL_FLAG_ALGORITHM_STANDBY);
  return SENtralA2_i2c_write_value(HOST_INTERFACE_CTRL_REG, val);
}

int SENtralA2_init(SENtralA2Iface *iface)
{
  iface->set_parameter = &SENtralA2_set_parameter;
  iface->get_parameter = &SENtralA2_get_parameter;
  iface->set_sns_rate = &SENtralA2_set_sns_rate;
  iface->self_test = &SENtralA2_self_test;
  iface->set_meta_event_ctrl = &SENtralA2_set_meta_event_ctrl;

  /* init all sensor event sizes to 0 */
  memset(iface->event_sizes, 0, sizeof(iface->event_sizes));

  /* set standard non-sensor type event sizes */
  /* timestamps */
  iface->event_sizes[SENSOR_TYPE_TIMESTAMP] = 3;
  iface->event_sizes[SENSOR_TYPE_TIMESTAMP_OVERFLOW] = 3;
  iface->event_sizes[SENSOR_TYPE_TIMESTAMP_WAKE] = 3;
  iface->event_sizes[SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE] = 3;

  /* meta events */
  iface->event_sizes[SENSOR_TYPE_META] = 4;
  iface->event_sizes[SENSOR_TYPE_META_WAKE] = 4;

  if (SENtralA2_is_ready() != PNI_YES) {
    SENA2_E("Unable to communicate with SENtral-A2\r\n");
    return SENA2_RET_ERROR;
  }

  SENtralA2_i2c_write_value(RESET_REQ_REG, 0x01);
  Clock_Wait(10);
  SENtralA2_i2c_write_value(CHIP_CONTROL_REG, 1);
  Clock_Wait(10);
  displayA2StatusRegisters();
  Clock_Wait(1000);

  return SENA2_RET_OK;
}

