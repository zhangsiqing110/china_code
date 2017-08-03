/**
* @file         parking_sensor.c
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

#include "SENtralA2.h"
#include "ble_clock.h"
#include "adc_monitor.h"
#include "mpc9808.h"
#include "rtc.h"
#include <string.h>
#include "parking_buffer.h"
#include "wwdg.h"

#include "radio.h"
#include "parking.h"
#include "parking_sensor.h"
#include "parking_module.h"
#include "parking_sensor_rti.h"
#include "parking_fsm.h"

//#define ENABLE_DBG_MSG
#ifdef ENABLE_DBG_MSG
#define PARKING_SENSOR_DBG  1
#else
#define PARKING_SENSOR_DBG  0
#endif

/*
 * Bitmask of which RTI meta events to report to cloud
 */
static const uint32_t PARK_SNS_META_EVENT_MASK = ((1 << META_EVENT_ERROR)
    | (1 << META_EVENT_SENSOR_EVENT)
    | (1 << META_EVENT_SELF_TEST_RESULT)
    | (1 << META_EVENT_INITIALIZED));

static PNI_ParkingPacket gLastCarPkt = { 0 };
static float gLastTemperature = 0.0f;
static float gLastBatteryStatus = 0.0f;

static int Parking_Ping(PNI_RadioIface *radio, PNI_ParkingBufferIface *buffer)
{
  PNI_RadioPong pong = { 0 };
  PNI_ParkingPacket pkt = {
    .sid = PARK_SNS_TYPE_PONG,
    .data = { 0 },
    .timestamp = gLastCarPkt.timestamp,
  };

  if (radio->ping(radio, &pong) != PNI_PARKING_RET_OK) {
      return PNI_PARKING_RET_ERROR;
  }

  PNI_PRINTF("!! PONG !! RSSI: %f, SNR: %f\r\n", pong.rssi, pong.snr);

  pkt.pong.rssi = pong.rssi;
  pkt.pong.snr = pong.snr;

  buffer->push(buffer, &pkt);

  return PNI_PARKING_RET_OK;
}

/**
 * @brief  function to return parking detection param IO size
 * @param uint8_t param IO number
 *
 * @retval Return indicate size of paramIO(1/2/4 = uint8_t/uint16_t/float)
 */
static uint8_t Parking_ParamIOSize(uint8_t paramIO)
{
  switch (paramIO) {
    case PARK_RTI_PIO_USER_CTX_INPUT:
    case PARK_RTI_PIO_USER_CAR_DETECTOR_OUTPUT_TYPE:
    case PARK_RTI_PIO_USER_G_ALARM_MODE:
    case PARK_RTI_PIO_USER_G_LS_RATE:
    case PARK_RTI_PIO_USER_G_HS_RATE:
    case PARK_RTI_PIO_USER_G_PARK_STRUCT_INIT_FLAG:
    case PARK_RTI_PIO_USER_TRIGGER_FLAG:
    case PARK_RTI_PIO_USER_BLE_TRIGGER_MODE:
    case PARK_RTI_PIO_USER_SHIPPING_MODE_FLAG:
    case PARK_RTI_PIO_USER_SHIPPING_MODE_INIT_FLAG:
    case PARK_RTI_PIO_USER_STANDBY_MODE_FLAG:
    case PARK_RTI_PIO_USER_TEMPERATURE:
    case PARK_RTI_PIO_USER_BLE_TRIGGER_ENABLE:
    case PARK_RTI_PIO_USER_LOCAL_BASE_LINE_UPDATE_ENABLE:
      return sizeof(uint8_t);

    case PARK_RTI_PIO_USER_CAL_TIMEOUT:
    case PARK_RTI_PIO_USER_CNT_THRD:
    case PARK_RTI_PIO_USER_STATE_CNT_THRD:
    case PARK_RTI_PIO_USER_STATE_CNT_THRD_TIMEOUT:
    case PARK_RTI_PIO_USER_TIMEOUT_TRANSITION:
    case PARK_RTI_PIO_USER_STANDBY_MODE_CNT:
    case PARK_RTI_PIO_USER_STANDBY_MODE_TIMEOUT:
      return sizeof(uint16_t);

    default:
      return sizeof(float);
  }
}

/**
 * @brief  function to parse data frame from Lora(XDot) interface
 * @param
 *
 * @retval Return indicate data send is confirmed (0/1 = OK/ERROR)
 */
uint32_t Parking_ParseFrameFromLora(PNI_RadioIface *radio,
        PNI_ParkingSensorIface *sensor, PNI_ParkingBufferIface *buffer)
{
  uint8_t cloudPkt[20] = { 0 };
  uint16_t byteRead = 0;
  uint8_t packetType = 0;
  int i = 0;

  int ret = PNI_PARKING_RET_OK;

  ret = radio->recv(radio, cloudPkt, 20);

  /* check for error */
  if (ret < 0) {
    return PNI_PARKING_RET_ERROR;
  }

  /* check for ok but no data recv */
  if (ret == 0) {
    PNI_PRINTF("[DBG] No data from Lora(%d)\r\n", byteRead);
    return PNI_PARKING_RET_OK;
  }

  /* data recv, parse and execute */
  byteRead = ret;
  packetType = cloudPkt[0];

  PNI_PRINTF("[DBG] Received data from Lora { bytes: %u, id: %u }\r\n",
      byteRead, packetType);

  if (byteRead > 1) {
    PNI_PRINTF("[DBG] Payload: ");
    for (i = 1; i < byteRead; i++) {
      PNI_PRINTF("0x%.02X,", cloudPkt[i]);
    }
    PNI_PRINTF("\r\n");
  }

  switch(packetType) {

    case PARK_CMD_RECALIBRATE:
      PNI_PRINTF("[DBG] Recalibrate Parking... \r\n");
      ret = sensor->recalibrate(sensor);
      break;

    case PARK_CMD_FORCE_OCCUPIED:
      PNI_PRINTF("[DBG] Force Occupied... \r\n");
      ret = sensor->force_occupied(sensor);
      break;

    case PARK_CMD_FORCE_VACANT:
      PNI_PRINTF("[DBG] Force Vacent... \r\n");
      ret = sensor->force_vacant(sensor);
      break;

    case PARK_CMD_SELF_TEST:
      // TODO:
      PNI_PRINTF("[DBG] Self Test... \r\n");
      ret = sensor->self_test(sensor);
      break;

    case PARK_CMD_INTERMEDIATE_REPORT_ENABLE:
      PNI_PRINTF("[DBG] Enable Intermediate State Reporting...\r\n");
      ret = sensor->set_inter_state_enabled(sensor, 1);
      break;

    case PARK_CMD_INTERMEDIATE_REPORT_DISABLE:
      PNI_PRINTF("[DBG] Disable Intermediate Reporting...");
      ret = sensor->set_inter_state_enabled(sensor, 0);
      break;

    case PARK_CMD_SET_WAKEUP_INTERVAL:
      PNI_PRINTF("[DBG] Set wakeup Interval... \r\n");
      if (byteRead != 3)
      {
        PNI_PRINTF("\tWrong packet size, size = %u\r\n", byteRead);
      }
      else
      {
        uint8_t * pByte = (uint8_t*) &parkingWakeInterval;
        pByte[0]=cloudPkt[1];
        pByte[1]=cloudPkt[2];

        PNI_PRINTF("\tSetting wakeup interval to %u sec\r\n", parkingWakeInterval);
        if (parkingWakeInterval < 10)
        {
          parkingWakeInterval = 10;
          PNI_PRINTF("\tInvalid value, set wakup interval to %u sec\r\n", parkingWakeInterval);
        }
        if (parkingWakeInterval > 43200)
        {
          parkingWakeInterval = 43200;
          PNI_PRINTF("\tInvalid value, set wakup interval to %u sec\r\n", parkingWakeInterval);
        }
        copyParkingWakeInterval = parkingWakeInterval;
      }
      break;

    case PARK_CMD_CAR_DETECTOR_DATA_ENABLE:
      PNI_PRINTF("[DBG] Enable Mag Data Reporting... \r\n");
      ret = sensor->set_car_detector_data_enabled(sensor, 1);
      break;

    case PARK_CMD_CAR_DETECTOR_DATA_DISABLE:
      PNI_PRINTF("[DBG] disable Mag Data Reporting\r\n");
      ret = sensor->set_car_detector_data_enabled(sensor, 0);
      break;

    case PARK_CMD_SET_PARAMETER: {
      #if PARKING_SENSOR_DBG
      PNI_PRINTF("[DBG] set Param IO\r\n");
      #endif
      float fValue;
      memcpy((uint8_t *)&fValue, &cloudPkt[3], sizeof(fValue));

      if (Parking_ParamIOSize(cloudPkt[2]) == sizeof(uint8_t)) {
        if (fValue > 255.0f) {
          PNI_PRINTF("[ERROR]incorrect unsigned char value, %f\r\n", fValue);
        } else {
          uint8_t u8Value = (uint8_t)fValue;
          #if PARKING_SENSOR_DBG
          PNI_PRINTF("\tpage: %u\r\n\tparameter: %u\r\n\tvalue: %u(0x%.02X)\r\n",
                    cloudPkt[1], cloudPkt[2], u8Value, u8Value);
          #endif
          ret = sensor->set_parameter(sensor, cloudPkt[1], cloudPkt[2],(uint8_t *) &u8Value, sizeof(u8Value));
        }
      } else if (Parking_ParamIOSize(cloudPkt[2]) == sizeof(uint16_t)) {
        if (fValue > 65535.0f) {
          PNI_PRINTF("[ERROR]incorrect unsigned short value, %f\r\n", fValue);
        } else {
          uint16_t u16Value = (uint16_t)fValue;
          #if PARKING_SENSOR_DBG
          PNI_PRINTF("\tpage: %u\r\n\tparameter: %u\r\n\tvalue: %u(0x%.04X)\r\n",
                    cloudPkt[1], cloudPkt[2], u16Value, u16Value);
          #endif
          ret = sensor->set_parameter(sensor, cloudPkt[1], cloudPkt[2],(uint8_t *) &u16Value, sizeof(u16Value));
        }
      } else {
        #if PARKING_SENSOR_DBG
        PNI_PRINTF("\tpage: %u\r\n\tparameter: %u\r\n\tvalue: %f\r\n",
                  cloudPkt[1], cloudPkt[2], fValue);
        #endif
        ret = sensor->set_parameter(sensor, cloudPkt[1], cloudPkt[2],(uint8_t *) &fValue, sizeof(float));
      }
    } break;

    case PARK_CMD_GET_PARAMETER: {
      #if PARKING_SENSOR_DBG
      PNI_PRINTF("[DBG] get Param IO\r\n");
      #endif
      PNI_ParkingPacket pkt = {
        .sid = PARK_SNS_TYPE_PARAM_IO,
        .data = { 0 },
        .timestamp = gLastCarPkt.timestamp,
      };

      float fValue;
      uint8_t u8Value;
      uint16_t u16Value;
      if (Parking_ParamIOSize(cloudPkt[2]) == sizeof(u8Value)) {
        ret = sensor->get_parameter(sensor, cloudPkt[1], cloudPkt[2], (uint8_t *)&u8Value, sizeof(u8Value));
        fValue = u8Value;
      } else if (Parking_ParamIOSize(cloudPkt[2]) == sizeof(u16Value)) {
        ret = sensor->get_parameter(sensor, cloudPkt[1], cloudPkt[2], (uint8_t *)&u16Value, sizeof(u16Value));
        fValue = u16Value;
      } else {
        ret = sensor->get_parameter(sensor, cloudPkt[1], cloudPkt[2], (uint8_t *)&fValue, sizeof(fValue));
      }

      if (ret < 0) {
        PNI_PRINTF("[ERROR] reading Param IO: page=%u, param=%u\r\n",
                  cloudPkt[1], cloudPkt[2]);
      } else {
        #if PARKING_SENSOR_DBG
        PNI_PRINTF("\tpage: %u\r\n\tparameter: %u\r\n\tvalue: %f\r\n",
                  cloudPkt[1], cloudPkt[2], fValue);
        #endif
        pkt.param_io.page = cloudPkt[1];
        pkt.param_io.param = cloudPkt[2];
        pkt.param_io.value = fValue;

        buffer->push(buffer, &pkt);
      }
    } break;

    case PARK_CMD_FULL_MAG_DATA_REPORT_START:
      PNI_PRINTF("[DBG] Start full mag data reporting...");
      if (byteRead != 3)
      {
        PNI_PRINTF(" Wrong packet size(%u)\r\n", byteRead);
      }
      else
      {
        uint8_t * pByte = (uint8_t*) &totalMagDataLogTime;
        pByte[0]=cloudPkt[1];
        pByte[1]=cloudPkt[2];

        PNI_PRINTF("stop in %d sec\r\n", totalMagDataLogTime);

        if (totalMagDataLogTime > 0)
        {
          RTC_DateTypeDef sdatestructureget;

          /* work around for PPX-22 bug - always disable car mag data first */
          sensor->set_car_detector_data_enabled(sensor, 0);

          /* enable the full mag data reporting(1hz and 4hz) */
          sensor->set_hw_alarm_mode_enabled(sensor, 0);     // disable hw alarm, so RTI won't filter any event
          sensor->set_car_detector_data_enabled(sensor, 1); // enable the car detect mag data

          /* track starting time */
          HAL_RTC_GetTime(&hrtc, &magDatalogTimeStruct, RTC_FORMAT_BIN);
          /* Get the RTC current Date */
          HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

          SET_FSM_FLAG(FSM_FLAG_MAG_LOG_START);
        }
      }
      break;

    case PARK_CMD_RESET:
      PNI_PRINTF("[DBG] Reset\r\n");
      PNI_PRINTF("\r\n\r\nRebooting system in 3 sec...\r\n");
      /* reset the board in 3 sec */
      Clock_Wait(3000);
      HAL_NVIC_SystemReset();
      break;

    case PARK_CMD_PING:
      PNI_PRINTF("[DBG] Ping\r\n");
      if (Parking_Ping(radio, buffer) != PNI_PARKING_RET_OK) {
          PNI_PRINTF("[ERROR] executing ping\r\n");
      }
      break;

    case PARK_CMD_ALARM_A_INTERVAL: /* temperature and battery wakeup interval */
      PNI_PRINTF("[DBG] Set AlarmA Interval... \r\n");
      if (byteRead != 3)
      {
        PNI_PRINTF("\tWrong packet size, size = %u\r\n", byteRead);
      }
      else
      {
        uint8_t * pByte = (uint8_t*) &gAlarmAInterval;
        pByte[0]=cloudPkt[1];
        pByte[1]=cloudPkt[2];

        PNI_PRINTF("\tSetting AlarmA interval to %u sec\r\n", gAlarmAInterval);
        if (gAlarmAInterval < 60)
        {
          gAlarmAInterval = 60;
          PNI_PRINTF("\tInvalid value, set AlarmA interval to %u sec\r\n", gAlarmAInterval);
        }

        // set the new AlarmA interval
        RTC_SetAlarmA(gAlarmAInterval);
      }
      break;

    case PARK_CMD_MFG_WAKE_BLE:
      PNI_PRINTF("[DBG] Mfg Wake BLE... size=%u, \r\n", byteRead);
      SET_FSM_FLAG(FSM_FLAG_BLE_WAKE);
      break;

    case PARK_CMD_RADIO_TXP: {
      uint16_t txp = (cloudPkt[2] << 8) | cloudPkt[1];
      PNI_PRINTF("[DBG] Setting radio TX power to %u dBm\r\n", txp);
      ret = PNI_PARKING_RET_OK;
      if (radio->set_tx_power(radio, (uint8_t)txp) != PNI_RADIO_RET_OK) {
        PNI_PRINTF("[ERROR] setting radio tx power\r\n");
        ret = PNI_PARKING_RET_ERROR;
      }
      break;
    }

    default:
      PNI_PRINTF("[ERROR] unknown packet type(%u)", packetType);
      ret = PNI_PARKING_RET_ERROR;
  }

  return ret;
}

/**
 * @brief  function to send data frame through Lora(XDot) interface to Cloud.
 * @param  uint8_t *  data
 *        uint16_t  data size
 *        uint16_t  pktCnt - packet_count number of packets in data
 *        int32_t * loraRsp - (output) confirmed - lora response
 *                                not confirmed - retry in msec
 *
 * @retval Return indicate data send is confirmed (0/1 = DISABLE(NO)/ENABLE(YES))
 */
FunctionalState Parking_SendFrame2Lora(PNI_ParkingBufferIface *parking_buffer,
    PNI_RadioIface *radio, int32_t *loraRsp)
{
  FunctionalState IsTxConfirmed;

  IsTxConfirmed = DISABLE;

  *loraRsp = radio->send(radio, parking_buffer);

  /* depend on return value, make appropriate action */
  if (*loraRsp == PNI_RADIO_RET_EBUSY)
  {
    IsTxConfirmed = DISABLE;

    #if PARKING_SENSOR_DBG
    PNI_PRINTF("[%d]Exit without waiting, let state machine hanlde other task...\r\n", *loraRsp);
    #endif

    /* since we still not confirmed the data - ask caller to retry in 100ms */
    *loraRsp = 100;
  }
  else if (*loraRsp == PNI_RADIO_RET_EINVAL)
  {
    PNI_PRINTF("[ERROR] error for Lora to send frame, ret = %d\r\n", *loraRsp);

    /* PPX-105: don't check recv frame if we have error */
    IsTxConfirmed = DISABLE;

    /* ask caller to retry in 100ms */
    *loraRsp = 100;
  }
  else if (*loraRsp >= 1000)
  {
    IsTxConfirmed = DISABLE;

    #if PARKING_SENSOR_DBG
    PNI_PRINTF("[%d]Exit without waiting, let state machine hanlde other task...\r\n", *loraRsp);
    #endif
  }
  else if ((*loraRsp >= 0) && (*loraRsp < 1000))
  {
    #if PARKING_SENSOR_DBG
    PNI_PRINTF("[%d]Confirmed...\r\n", *loraRsp);
    #endif

    IsTxConfirmed = ENABLE;
  }
  else
  {
    PNI_PRINTF("[ERROR] Unknown XDot return value, ret = %d\r\n", *loraRsp);

    /* PPX-105: assume unknown XDot return cmd means error case */
    IsTxConfirmed = DISABLE;

    /* ask caller to retry in 100ms */
    *loraRsp = 100;
  }

  return IsTxConfirmed;
}

/**
 * @brief  function to check for data from cloud. XDot require send packet before it can check
 *       for received packet. This function only start the checking process, the command from
 *       cloud will automatic gets handled from Parking_ParseFrameFromLora, when confirmed
 *       packet sent.
 * @param  PNI_ParkingBufferIface *parking_buffer
 *
 * @retval Return none
 */
void Parking_CheckRcvFrameFromLora(PNI_ParkingBufferIface *parking_buffer)
{
  /*
   * if parking buffer is empty, send last parking detector packet
   */
  if (parking_buffer->is_empty(parking_buffer)) {
    parking_buffer->push(parking_buffer, &gLastCarPkt);
  }
}

#if ENABLE_MCU_ADC
static int Parking_UpdateBattery()
{
  BATT_MEANS_PWR_ON();

  /* start monitor battery and temperature - interrupt */
  Adc_Monitor_Start();

  /* give it couple msec to get battery status*/
  Clock_Wait(3);

  PNI_PRINTF("[INFO]Reading Battery(ADC8)...");
  gLastCarPkt.car_detector.battery = gLastBatteryStatus = Battery_Monitor_Read();
  PNI_PRINTF(" %f\r\n", gLastCarPkt.car_detector.battery);

  BATT_MEANS_PWR_OFF();

  return 0;
}
#endif

#if ENABLE_MCP9808
static int Parking_UpdateTemperature(PNI_ParkingSensorIface *sensor,
        PNI_ParkingTemperatureSensorIface *tmp_sns)
{
    /* don't get temperature if sensor is not present */
    if (!tmp_sns) {
        PNI_PRINTF("[ERROR] Temperature sensor not found!\r\n");
        return PNI_PARKING_RET_ERROR;
    }

    /* get temperature */
    if (tmp_sns->get_temperature(tmp_sns, &gLastTemperature)
            != PARK_TEMP_SNS_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

    PNI_PRINTF("[INFO] Read temperature: %f\r\n", gLastTemperature);

    /* update car detector packet */
    gLastCarPkt.car_detector.temperature = gLastTemperature;

    /* set temperature in parking sensor */
    return sensor->set_temperature(sensor, (int8_t)gLastTemperature);
}
#endif /* ENABLE_MCP9808 */

/**
 * @brief  function to handle RTC Alarm A - every hourly at RTC clock xx:08:35
 * @param none
 *
 * @retval Return none
 */
void Parking_AlarmAHandler(PNI_ParkingSensorIface *sensor,
        PNI_ParkingTemperatureSensorIface *tmp_sns)
{

#if ENABLE_RTC_ALARM
  if (rtcAlarmAFlag == 1)
  {
    PNI_PRINTF("[DBG]RTC Alarm A...\r\n");
  /*** get temperature reading and Battery using MCU ADC ***/
#if ENABLE_MCU_ADC
    if (Parking_UpdateBattery() != PNI_PARKING_RET_OK)
    {
        PNI_PRINTF("[ERROR] Error updating battery\r\n");
    }
#endif  // ENABLE_MCU_ADC

  /*** get temperature reading from MPC9808 ***/
#if ENABLE_MCP9808
    if (Parking_UpdateTemperature(sensor, tmp_sns) != PNI_PARKING_RET_OK) {
        PNI_PRINTF("[ERROR] Error updating temperature\r\n");
    }
#endif /* ENABLE_MCP9808 */

    // setup the next alarm
    RTC_SetAlarmA(gAlarmAInterval);

    rtcAlarmAFlag = 0;
  }
#endif

}

static void Parking_PushPacket(PNI_ParkingBufferIface *parking_buffer,
    SENtralA2_SensorPacket *sns)
{
  PNI_ParkingPacket pkt = {
    .sid = sns->sid,
    .data = { 0 },
    .timestamp = sns->timestamp,
  };
  uint8_t ignore = 0;

  memcpy(pkt.data, sns->data.bytes, MIN(sizeof(pkt.data), sns->count));

  switch (sns->sid) {
    case SENSOR_TYPE_CAR_DETECTOR: {
      /* hack in the temperature */
      pkt.car_detector.temperature = gLastTemperature;
      pkt.car_detector.battery= gLastBatteryStatus;

      /* store last car state */
      gLastCarPkt = pkt;

      PNI_PRINTF("[%u] Car Detect: 0x%02X, 0x%02X\r\n", pkt.timestamp,
          pkt.car_detector.car_presence, pkt.car_detector.flags);

      /* PPX-112: check for patterns detected by RTI module */
      if (DETECT_PATTERN(pkt.car_detector.flags, CAR_DET_FLAG_BLE_WAKE_UP)) {
        #if PARKING_SENSOR_DBG
        PNI_PRINTF("[DBG] Pattern A - BLE Wake Up!!!\r\n");
        #endif
        SET_FSM_FLAG(FSM_FLAG_BLE_WAKE);
      } else if (DETECT_PATTERN(pkt.car_detector.flags, CAR_DET_FLAG_DISABLE_SHIPPING_MODE)) {
        #if PARKING_SENSOR_DBG
        PNI_PRINTF("[DBG] Pattern B - Out of Shipping mode!!!\r\n");
        #endif

        // TODO: Need to check, RTI not sending pattern B flag when out-of-shipping
        CLEAR_FSM_FLAG(FSM_FLAG_SHIPPING_MODE);
      } else if (DETECT_PATTERN(pkt.car_detector.flags, CAR_DET_FLAG_SHIPPING_MODE)) {
        #if PARKING_SENSOR_DBG
        PNI_PRINTF("[DBG] Pattern D - Shipping mode!!!\r\n");
        #endif
        SET_FSM_FLAG(FSM_FLAG_SHIPPING_MODE);
      } else {
        /* other patterns we don't care */
      }
      break;
    }

    case SENSOR_TYPE_CAR_MAG_DATA: {
      PNI_PRINTF("[%u] Car Detect Mag Data = %3.3f,%3.3f,%3.3f\r\n",
          pkt.timestamp, pkt.car_detector_data.x, pkt.car_detector_data.y,
          pkt.car_detector_data.z);
      break;
    }

    case SENSOR_TYPE_META:
    case SENSOR_TYPE_META_WAKE:
      /* if meta event type not in mask, don't push to cloud */
      ignore = (((1 << (uint32_t)pkt.meta_event.type)
          & PARK_SNS_META_EVENT_MASK) == 0);
      break;
  }

  if (!ignore) {
    parking_buffer->push(parking_buffer, &pkt);
  }
}

int Parking_ParseFifo(SENtralA2Iface *rti, uint8_t *buffer, uint32_t size,
    PNI_ParkingBufferIface *parking_buffer)
{
  SENtralA2_SensorPacket sns = { 0 };
  int ret = SENtralA2_parse_fifo(rti, buffer, size, &sns);

  if (ret < 1) {
    PNI_PRINTF("End of FIFO or unknown sensor received: %d\r\n", ret);
    return size;
  }

  switch (sns.sid) {
    case SENSOR_TYPE_TIMESTAMP:
    case SENSOR_TYPE_TIMESTAMP_OVERFLOW:
    case SENSOR_TYPE_TIMESTAMP_WAKE:
    case SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE:
      /* don't push timestamp packets */
      goto exit;
  }

  Parking_PushPacket(parking_buffer, &sns);

exit:
  return (sns.count + 1);
}

