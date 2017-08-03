/**
* @file         parking_fsm.c
*
* @brief        PNI Parking Finite State Machine
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

/* Includes ------------------------------------------------------------------*/
#include "parking_fsm.h"
#include "pni_config.h"
#include "stm32l0xx_hal.h"
#include "SENtralA2.h"
#include "rtc.h"
#include "ble_clock.h"
#include "wwdg.h"
#include "parking_sensor.h"
#include "parking_module.h"
#include "pni_device_info.h"
#include "pni_fota.h"
#include "xdot.h"

#if ENABLE_XDOT_RADIO
#include "crypto.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static PNI_ParkingState gParkState = PARK_STATE_INIT ;
/* WakeUp Interval - default 300 sec, configurable via cloud(10 ~ 43200 sec)
 */
uint16_t parkingWakeInterval = 300;

uint16_t copyParkingWakeInterval = 300;

/* AlarmA Interval - default 3600 sec, configurable via cloud()
 *  - Temperature
 *  - Battery
 */
uint16_t gAlarmAInterval = 3600;

static FunctionalState txConf = ENABLE;    //Keeps track of last LoRa transmit status
static uint16_t bytesRead = 0;
static uint16_t bytesLeftToRead = 0;
static uint8_t  SENtralA2FIFOBuf[3000];    // temp buffer to store A2 FIFO, bump up to 3000, this should be more than enough to handle Parking FIFO
static uint16_t bytesAvailable = 0;
uint16_t totalMagDataLogTime = 0;
RTC_TimeTypeDef magDatalogTimeStruct = { 0 };
uint8_t fsmFlag = 0x0;

#if ENABLE_PRINTF
#define PARKING_STATE_PRINT (1)
#else
#define PARKING_STATE_PRINT (0)
#endif

#if PARKING_STATE_PRINT
static PNI_ParkingState m_LastParkState = PARK_STATE_INIT;
static const char *PARK_STATE_STRINGS[PARK_STATE_MAX] = {
    [PARK_STATE_INIT] = "Init",
    [PARK_STATE_STOP] = "Stop",
    [PARK_STATE_DATA_READY] = "Data Ready",
    [PARK_STATE_SLEEP] = "Sleep",
    [PARK_STATE_GETFIFO] = "Get A2 FIFO",
    [PARK_STATE_PARSE] = "Parse A2 FIFO",
    [PARK_STATE_TX_LORA] = "TX LoRa",
    [PARK_STATE_MFG] = "Manufacturing Mode",
    [PARK_STATE_MFG_LORA_TEST] = "MFG Lora Test",
    [PARK_STATE_MFG_BLE_TEST] = "MFG BLE Test",
    [PARK_STATE_MFG_TEST_DONE] = "Manufacturing Done",
    [PARK_STATE_MAINT] = "Maintenance Mode",
    [PARK_STATE_BLE_ADVER] = "BLE Advertising",
    [PARK_STATE_BLE_CONN] = "BLE Connected",
    [PARK_STATE_SHIP] = "Shipping Mode",
    [PARK_STATE_UPGRADE] = "Upgrade Mode",
};
#endif /* SENTRAL_A2_STATE_PRINT */

extern PNI_DeviceInfo gPniDeviceInfo;

#if ENABLE_XDOT_RADIO
/*
* PNI LoRa manufacturing
*/
static uint8_t PNI_LORA_MFG_APP_EUI[] = {
   0x14, 0x93, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00
};
static char gPniLoraMfgAppEui[sizeof(PNI_LORA_MFG_APP_EUI) * 3];
static uint8_t PNI_LORA_MFG_APP_KEY[16] = { 0 };
static char gPniLoraMfgAppKey[sizeof(PNI_LORA_MFG_APP_KEY) * 3];

/*
 * Manfacturing mode xDot configuration
 */
static Xdot_Cfg m_XdotCfgMfg = {
  .echo_enabled = 1,
  .ism_band = XDOT_ISM_BAND_US915,
  .freq_sub_band = XDOT_FSB_902_3__903_7,   // temporary until gateway able to set correct sub band
  //.freq_sub_band = XDOT_FSB_903_9__905_3,   // Manufacture sub band
  .public_network_enabled = 1,
  .network_join_mode = XDOT_NJM_OTA,
  .network_app_eui = NULL,
  .ni_type = XDOT_NI_TYPE_HEX,
  .network_app_key = NULL,
  .nk_type = XDOT_NI_TYPE_HEX,
  .sf = XDOT_SF_7_BW_125,
  .acks_enabled = 1,
  .rx_type = XDOT_RX_OUT_TYPE_HEX,
  .txp = 11, /* TX power = 11 dBm */
};

#endif /* ENABLE_XDOT_RADIO */

#if TEST_BLE_FOTA_STATE
static uint8_t checkBleFlagCnt = 0;
#endif
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
extern void SystemClock_Config(void);

/**
 * @brief Function to put MCU to stop mode and wait for interrupt. This function will restore MCU
 *      back to state before enter stop mode
 *
 * @param PNI_ParkingBufferIface parkBufIf
 *        PNI_ParkingState current state
 *        PNI_ParkingState * nextState(output)
 *
 * @retval int8_t Return value for checking  (1/-1/-2 == Ok/Error/radio not sleep)
 */
static int8_t Mcu_Stop_WFI(PNI_ParkingModule *module,
        PNI_ParkingState curState, PNI_ParkingState * nextState,
        PNI_RadioIface *radio)
{
  int8_t result = FSM_STATUS_OK;

#if ENABLE_MCU_STOP_MODE

  #if ENABLE_XDOT_SLEEP
  if (radio->sleep(PNI_RADIO_SLEEP_MODE_STOP) != PNI_RADIO_RET_OK)
  {
    //PNI_PRINTF("[ERROR] Putting xDot to sleep for %u sec\r\n", parkingWakeInterval);
    result = FSM_STATUS_RADIO_NOT_SLEEP;
  }
  #endif /* ENABLE_XDOT_SLEEP */

  HAL_SPI_DeInit(&hspi1);
  HAL_SPI_DeInit(&hspi2);

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();

  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();

  /* PPC-58: memory pins below need reconfigure to optimize power consumption */
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.Pin = MEM_PCTRL_Pin;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEM_PCTRL_GPIO_Port, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = MEM_CSN_Pin | MEM_SCK_Pin;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  if ((CHECK_FSM_FLAG(FSM_FLAG_SHIPPING_MODE) == 0) ||
       (result == FSM_STATUS_RADIO_NOT_SLEEP))
  {
    PNI_PRINTF("[FSM] sleep for %d sec\r\n", parkingWakeInterval);
    //PNI_PRINTF("Jobs done, time to deep sleep!!\n");
    RTC_ConfigureWakeUpTimerSec(parkingWakeInterval);
  }

  // PPX-62 and BIST might have A2 interrupt before enter STOP mode
  if (A2Interrupt == 0)
  {
    /*** go to STOP MODE now ***/
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	PNI_PRINTF("szhang add 11111111111\r\n");
  }

  /*** Configure system back to NORMAL MODE ***/
  SystemClock_Config();

  /* make sure we restore all the interface back */
  HAL_GPIO_WritePin(GPIOA, MEM_PCTRL_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStructure.Pin = MEM_PCTRL_Pin;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = MEM_CSN_Pin;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = MEM_SCK_Pin;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  MX_SPI1_Init();
  MX_SPI2_Init();
  sFLASH_Init();

  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  #if ENABLE_WAKEUP_TIME
  RTC_CalendarShow();
  #endif  // ENABLE_CLOCK_DISPLAY

#endif  // ENABLE_MCU_STOP_MODE

/* Check what wakeup device from STOP mode */
  if (A2Interrupt != 0)
  {
    /* check for mfg testing */
    if (CHECK_FSM_FLAG(FSM_FLAG_SHIPPING_MODE))
    {
      if (module->sensor->is_mfg_triggered(module->sensor) == 1)
      {
        HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
        *nextState = PARK_STATE_MFG;
      }
      else
      {
        /* RTI doesn't show pattern B flag when out-of-shipping, for now clear the shipping flag here */
        CLEAR_FSM_FLAG(FSM_FLAG_SHIPPING_MODE);

        *nextState = PARK_STATE_DATA_READY;
      }
    }
    else
    {
      //PNI_PRINTF("RTI Module!!\r\n");
      *nextState = PARK_STATE_DATA_READY;
    }
  }
/* this is configurable wakeup timer from Cloud - default start every 10 sec */
  else if ((rtcWakeUpFlag == 1) || (rtcAlarmAFlag == 1))
  {
    if (CHECK_FSM_FLAG(FSM_FLAG_SHIPPING_MODE))
    {
      /* if in shipping mode, and XDot still not sleeping */
      if (result == FSM_STATUS_RADIO_NOT_SLEEP)
      {
        //PNI_PRINTF("[DBG] Shipping mode, Radio not sleep, try put the radio back to sleep again\r\n");
        *nextState = PARK_STATE_TX_LORA;
      }

      /* if in shipping mode, and XDot sleeping */
      else
      {
        /* we should not send anything stay in STOP */
        *nextState = PARK_STATE_STOP;
      }
    }
    else
    {
      /* only check for lora when not in shipping mode */
      Parking_CheckRcvFrameFromLora(module->buffer);

      // we added dummy packet to the queue, move to TX_LORA to start the process
      *nextState = PARK_STATE_TX_LORA;
    }
    rtcWakeUpFlag = 0;
  }
  else
  {
    PNI_PRINTF("UNKNOWN INTERRUPT!!\r\n");

    /* if we don't know what interrupt us, then keep looping in this state */
    *nextState = PARK_STATE_STOP;
  }

  return result;
}


/**
 * @brief  Put Mcu to Sleep and wait for amount of time.
 * @param  uint32_t sleepMs
 * @retval None
 */
static void Mcu_Sleep_Ms(uint32_t sleepMs)
{
  uint32_t trackTime = Clock_Time();
  //PNI_PRINTF("[DBG] Clock wait 2: i = %u\r\n", i);

  while (sleepMs > 1)
  {
    #if ENABLE_WWDG
    if (HAL_WWDG_Refresh(&hwwdg) != HAL_OK)
          Error_Handler();
    #endif

    if (sleepMs < 500)
      RTC_ConfigureWakeUpTimerUSec(sleepMs);
    else
      RTC_ConfigureWakeUpTimerUSec(500);

    /* go to sleep - Should not go to stop mode, because Host need to be active to receive data */
    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();

    /* Disable Wakeup Counter */
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    if (rtcWakeUpFlag)
      rtcWakeUpFlag = 0;
#if 0
    if (A2Interrupt != 0)
    {
      // if we are wake up by SENtral, need to exist wait loop and handle A2 event
      i = 0;
    }
#endif
    //PNI_PRINTF("[DBG] Clock wait 2: wake up for work - clock = %u\r\n", Clock_Time());

    if (sleepMs < 500)
      sleepMs = 0;
    else
      sleepMs -= 500;
  }

  #if ENABLE_WWDG
  if (HAL_WWDG_Refresh(&hwwdg) != HAL_OK)
    Error_Handler();
  #endif
}


void Parking_SetFsm(PNI_ParkingState state)
{
  gParkState = state;
}

void Parking_Fsm(PNI_ParkingModule *module)
{
  SENtralA2Iface *rti = (SENtralA2Iface *)module->sensor->params;
#if PARKING_STATE_PRINT
  if ((gParkState < PARK_STATE_MAX) && (gParkState != m_LastParkState)) {
    PNI_PRINTF("[FSM] %s => %s\r\n", PARK_STATE_STRINGS[m_LastParkState],
        PARK_STATE_STRINGS[gParkState]);
    m_LastParkState = gParkState;
  }
#endif /* SENTRAL_A2_STATE_PRINT */

  switch (gParkState)
  {
    case PARK_STATE_INIT:
    {
      /* default the wakeup checking interval to 300 sec - this is configurable via cloud */
      copyParkingWakeInterval = parkingWakeInterval = 300;

      /* default the AlarmA interval to 3600 sec - this is configurable via cloud */
      gAlarmAInterval = 3600;

      /* if we can get here, MCU image is stable image */
      OtaDfuUpdateMcuOtaImgNum();

#if ENABLE_RTC_ALARM
      rtcAlarmAFlag = 1;
      /* update battery and temperature in init */
      Parking_AlarmAHandler(module->sensor, module->tmp_sns);

      CLEAR_FSM_FLAG(FSM_FLAG_ALARM_0_STOP);//isAlarmAStop = 0;
#endif
      /* PPX-98 */
      fsmFlag = 0;

      gParkState = PARK_STATE_DATA_READY;
      break;
    }

    case PARK_STATE_TX_LORA:
    {
      int32_t loraRsp = 0;

      if (CHECK_FSM_FLAG(FSM_FLAG_MAG_LOG_START))
      {
        if (RTC_TimeExpired(magDatalogTimeStruct, totalMagDataLogTime))
        {
          //PNI_PRINTF("Finish Tx Mag Data = %u usec\r\n", Clock_Time() - fullMagDataStartTime);

          /* done with mag data reporting(1hz and 4hz) - reset eveything back to normal */
          module->sensor->set_hw_alarm_mode_enabled(module->sensor, 1);     // enable hw alarm
          module->sensor->set_car_detector_data_enabled(module->sensor, 0); // disable the car detect mag data

          magDatalogTimeStruct.Hours = 0;
          magDatalogTimeStruct.Minutes = 0;
          magDatalogTimeStruct.Seconds = 0;
          totalMagDataLogTime = 0;
          CLEAR_FSM_FLAG(FSM_FLAG_MAG_LOG_START);
        }
      }

      /* if queue is not empty, send to LoRa */
      if (!module->buffer->is_empty(module->buffer)) {
            txConf = Parking_SendFrame2Lora(module->buffer, module->radio, &loraRsp);
      }

      /* PPX-110: prevent loop in XDot */
      if (CHECK_FSM_FLAG(FSM_FLAG_BLE_WAKE))
      {
        gParkState = PARK_STATE_MAINT;
      }
      else if (txConf)
      {

        /* restore the wake timer back to normal */
        if (copyParkingWakeInterval != parkingWakeInterval)
        {
          //PNI_PRINTF("[DBG] restore wake interval from %u sec to %u sec\r\n", parkingWakeInterval, copyParkingWakeInterval);
          parkingWakeInterval = copyParkingWakeInterval;
        }

        /* check for recv msg */
        Parking_ParseFrameFromLora(module->radio, module->sensor,
            module->buffer);

        /* when queue is empty, go back to STOP state */
        if (module->buffer->is_empty(module->buffer))
        {
          #if ENABLE_RTC_ALARM
          /* ONLY when Lora is connected, handle alarmA */
          Parking_AlarmAHandler(module->sensor, module->tmp_sns);
          #endif  // ENABLE_RTC_ALARM

          //PNI_PRINTF("Confirmed Queue Empty... Switching state to STOP.\r\n");
          gParkState = PARK_STATE_STOP;
        }
      }
      else
      {
        #if SENTRAL_A2_DBG
        PNI_PRINTF("Tx Not Confirmed, lora ret = %d\r\n", loraRsp);
        #endif

        if (loraRsp >= 1000)
        {
          /* since we are going to wait for long time, go to stop mode and come back later */
          gParkState = PARK_STATE_STOP;

          /* make sure we set the parkingWakeInterval to the time we want to wake up in stop mode */
          parkingWakeInterval = loraRsp/1000;
        }
        else
        {
          Mcu_Sleep_Ms(loraRsp);

          if (A2Interrupt != 0)
          {
            gParkState = PARK_STATE_DATA_READY;
          }
          else
          {
            gParkState = PARK_STATE_TX_LORA;
          }
        }
      }

      break;
    }

    case PARK_STATE_STOP:
    {
      /* check for shipping mode flag */
      if (CHECK_FSM_FLAG(FSM_FLAG_SHIPPING_MODE))
      {
        PNI_PRINTF("[DBG]<<<< IN SHIPPING MODE >>>>>\r\n");

        #if ENABLE_RTC_ALARM
        SET_FSM_FLAG(FSM_FLAG_ALARM_0_STOP);
        #endif

        HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
      }
      else
      {
        #if ENABLE_RTC_ALARM
        if (CHECK_FSM_FLAG(FSM_FLAG_ALARM_0_STOP))
        {
          PNI_PRINTF("[DBG]<<<< OUT OF SHIPPING MODE >>>>>\r\n");

          /* just wake up from shipping mode
           * update temperature and set the alarm A clock
           * do this only 1 time
           */
          CLEAR_FSM_FLAG(FSM_FLAG_ALARM_0_STOP);
          rtcAlarmAFlag = 1;
          Parking_AlarmAHandler(module->sensor, module->tmp_sns);
        }
        #endif
      }

      /* PPX-98: Allow Cloud to wake up BLE */
      if (CHECK_FSM_FLAG(FSM_FLAG_BLE_WAKE))
      {
        /* move directly to maintenance state */
        gParkState = PARK_STATE_MAINT;
      }
      else
      {
        Mcu_Stop_WFI(module, PARK_STATE_STOP, &gParkState, module->radio);
      }
      break;
    }

    case PARK_STATE_DATA_READY:
    {
      //uint8_t enableBits = 255;
      //ParamInfo param = { 41, 1 };

		  //SENtralA2_param_read((uint8_t*)&enableBits, 14, &param, 1);
		  //PNI_PRINTF("Shipping mode during data ready paramio readback value =%u\r\n", enableBits);
      //Send data over to RTI through paramIO
      SENtralA2_getBytesRemaining(&bytesAvailable);
      gParkState = PARK_STATE_SLEEP;
      break;
    }

    case PARK_STATE_SLEEP:
    {
    #if 0
      /* Configure the WakeUp clock source */
      __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

      HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
      /* Sleep until I2C finish getting the data */
    #else
      /*
       * to save more power, need to go sleep if I2C taking too long
       * I2C DMA mode seems to cause problem, for now using I2C blocking mode
       */
      gParkState = PARK_STATE_GETFIFO;
    #endif
      break;
    }

    case PARK_STATE_GETFIFO:
    {
      //PNI_PRINTF("%u", A2Interrupt);
      if (bytesAvailable > 0)
      {
        //PNI_PRINTF("-%u", bytesAvailable);
        bytesLeftToRead = bytesAvailable;
        bytesAvailable = 0;
      }

      if (bytesLeftToRead > 0)
      {
        uint16_t bytesToRead;

        bytesToRead = bytesRead % 50 + I2C_MAX_READ > 50 ? 50 - bytesRead % 50 : I2C_MAX_READ;
        bytesToRead = MIN(bytesLeftToRead, bytesToRead);
        if (SENtralA2_i2c_read(bytesRead % 50, &SENtralA2FIFOBuf[bytesRead], bytesToRead) != SENA2_RET_OK)
        {
          //SENtralA2_unlock_mutex(paramIoMutex);
          PNI_PRINTF("[error]i2c read\r\n");
        }

        bytesLeftToRead -= bytesToRead;
        bytesRead += bytesToRead;

        gParkState = PARK_STATE_SLEEP;
      }
      else
      {
        A2Interrupt--;
        bytesLeftToRead = 0;
        // got all Fifo data ready for parsing
        gParkState = PARK_STATE_PARSE;
      }
      break;
    }

    case PARK_STATE_PARSE:
    {
      uint32_t byteUsed = 0;
      uint32_t index = 0;
      while (bytesRead > 0)
      {
        byteUsed = Parking_ParseFifo(rti, &SENtralA2FIFOBuf[index], bytesRead,
            module->buffer);
        index += byteUsed;
        bytesRead -= byteUsed;
      }
      bytesRead = 0;
      //printf("%u", A2Interrupt);

    /* If we have a lot of data, we should try dequeu some data, before we continue to process A2 */
      if (!module->buffer->is_empty(module->buffer))
      {
        gParkState = PARK_STATE_TX_LORA;
      }
      else if (A2Interrupt > 0)
      {
        gParkState = PARK_STATE_DATA_READY;
      }
      else
      {
        gParkState = PARK_STATE_STOP;
      }
      break;
    }

  /*******************************************************************/
  /*   Manufacture Mode - Lora and BLE testing                               */
  /*******************************************************************/
    case PARK_STATE_MFG:
      /* initialize any variable for manufacturing testing here */
      CLEAR_FSM_FLAG(FSM_FLAG_BLE_WAKE);

      set_connectable = TRUE;

      /* PPX-43 */
      DBG_LED_ON();

      // PPC-70: if eeprom config exist, use eeprom
      if ( CHECK_XDOT_FLAG_SET(XDOT_FLAG_MFG_EUI) )
        memcpy((uint8_t *)PNI_LORA_MFG_APP_EUI, EEPROM_U8_XDOT_MFG_EUI, sizeof(PNI_LORA_MFG_APP_EUI));

      /* PPX-36: Lora information for the Manufacture Network */
      // convert network id byte array to string
      xdot_bytes_to_hex(PNI_LORA_MFG_APP_EUI, sizeof(PNI_LORA_MFG_APP_EUI),
         gPniLoraMfgAppEui, ':', 0);

      // generate network key byte array using hmac_md5(network id, device id)
      hmac_md5(PNI_LORA_MFG_APP_EUI, sizeof(PNI_LORA_MFG_APP_EUI),
         (uint8_t *)&gPniDeviceInfo.radio_id, sizeof(gPniDeviceInfo.radio_id), PNI_LORA_MFG_APP_KEY);

      // convert network key byte array to string
      xdot_bytes_to_hex(PNI_LORA_MFG_APP_KEY, sizeof(PNI_LORA_MFG_APP_KEY),
         gPniLoraMfgAppKey, ':', 0);

      // assign string versions to xDot cfg
      m_XdotCfgMfg.network_app_eui = gPniLoraMfgAppEui;
      m_XdotCfgMfg.network_app_key = gPniLoraMfgAppKey;

      /* set mfg mode xDot configuration */
      if (module->radio->configure(module->radio, &m_XdotCfgMfg)
          != PNI_RADIO_RET_OK) {
        PNI_PRINTF("[ERROR] configuring radio for mfg mode\r\n");
      }

      gParkState = PARK_STATE_MFG_LORA_TEST;
      break;

    case PARK_STATE_MFG_LORA_TEST:
    {
      int32_t loraRsp = 0;
      /* if queue is not empty, send to LoRa */
      if (!module->buffer->is_empty(module->buffer)) {
          txConf = Parking_SendFrame2Lora(module->buffer, module->radio, &loraRsp);
      }

      if (txConf)
      {
        PNI_PRINTF("Tx Confirmed\r\n");

        // TODO: Check this later, if buffer not empty, cloud command doesn't get cleared
        // Temporary work around, wait until buffer are cleared
        if (module->buffer->is_empty(module->buffer))
        {
          /* check for recv msg */
          Parking_ParseFrameFromLora(module->radio, module->sensor,
              module->buffer);
        }

        if (CHECK_FSM_FLAG(FSM_FLAG_BLE_WAKE))
        {
          gParkState = PARK_STATE_MFG_BLE_TEST;
        }
        else
        {
          Mcu_Sleep_Ms(5000);
          Parking_CheckRcvFrameFromLora(module->buffer);
        }
      }
      else
      {
  #if SENTRAL_A2_DBG
        PNI_PRINTF("Tx Not Confirmed, lora ret = %d\r\n", loraRsp);
  #endif
        /* for manufacture testing if lora return long wait - we don't want to wait that long */
        if (loraRsp > 500)
        {
          PNI_PRINTF("Tx Not Confirmed, lora ret = %d\r\n", loraRsp);
          loraRsp = 500;
        }

        Mcu_Sleep_Ms(loraRsp);

        //gParkState = PARK_STATE_MFG_LORA_TEST;
      }
    }
      break;

    case PARK_STATE_MFG_BLE_TEST:
      /* PPX-43 */
      DBG_LED_OFF();

      if (set_connectable)
      {
        setConnectable(&gPniDeviceInfo.radio_id);
        set_connectable = FALSE;
      }

      /* Wait for Event */
      __WFI();

      if(HCI_ProcessEvent)
      {
        /* wait for BLE to connect */
        HCI_ProcessEvent=0;
        HCI_Process();
      }
      break;

    case PARK_STATE_MFG_TEST_DONE:
#if ENABLE_WWDG
      if (HAL_WWDG_Refresh(&hwwdg) != HAL_OK)
        Error_Handler();
#endif

      CLEAR_FSM_FLAG(FSM_FLAG_BLE_WAKE);
#if 0
      /* restore normal xDot configuration */
      if (module->radio->configure(module->radio, &m_LastXdotCfg)
          != PNI_RADIO_RET_OK) {
        PNI_PRINTF("[ERROR] configuring radio for normal mode\r\n");
      }
#endif
      /* Probably reboot system would be more reliable after finish mfg testing */
      PNI_PRINTF("[WARNING] Maintenance Mode, no activity timeout, reboot in 2 sec!!!\r\n");
      Clock_Wait(2000);
      HAL_NVIC_SystemReset();
      break;

  /*******************************************************************/
  /*   Maintenance Mode - Enable BLE and wait for App to connect     */
  /*******************************************************************/
    case PARK_STATE_MAINT:
    {
      // TODO: need to come back here, add handler for these state
      // TODO: should we disable LORA?
      // TODO: should we disable A2?
      // safety to prevent hang in BLE maintenance mode
      RTC_ConfigureWakeUpTimerSec(OTA_DFU_MAX_NO_ACTIVITY_SEC);

      set_connectable = TRUE;
      rtcWakeUpFlag = 0;

      /* PPX-98 */
      /* WORKAROUND PARK-261: Clear the EnableBLE command from cloud queue */
      int32_t loraRsp = 0;
      Parking_CheckRcvFrameFromLora(module->buffer);
      Parking_SendFrame2Lora(module->buffer, module->radio, &loraRsp);

      CLEAR_FSM_FLAG(FSM_FLAG_BLE_WAKE);

      gParkState = PARK_STATE_BLE_ADVER;
      break;
    }

    case PARK_STATE_BLE_ADVER:
      if (set_connectable)
      {
        setConnectable(&gPniDeviceInfo.radio_id);
        set_connectable = FALSE;
      }

      /* Wait for Event */
      __WFI();

      if(HCI_ProcessEvent)
      {
        /* wait for BLE to connect */
        HCI_ProcessEvent=0;
        HCI_Process();
        RTC_ConfigureWakeUpTimerSec(OTA_DFU_MAX_NO_ACTIVITY_SEC);
      }

      if (rtcWakeUpFlag == 1)
      {
        /* if here, no activity reboot the board */
        PNI_PRINTF("[WARNING] Maintenance Mode, no activity timeout, reboot in 2 sec!!!\r\n");
        Clock_Wait(2000);
        HAL_NVIC_SystemReset();
      }

      if (W2ST_CHECK_CONNECTION(W2ST_CONNECT_DFU_TERM))
      {
        gParkState = PARK_STATE_BLE_CONN;
      }
      else
      {
        gParkState = PARK_STATE_BLE_ADVER;
      }
      break;

    case PARK_STATE_BLE_CONN:
      /* Wait for Event */
      __WFI();

      if(HCI_ProcessEvent)
      {
        /* wait for FOTA to start */
        HCI_ProcessEvent=0;
        HCI_Process();
        RTC_ConfigureWakeUpTimerSec(OTA_DFU_MAX_NO_ACTIVITY_SEC);
      }

      if (rtcWakeUpFlag == 1)
      {
        /* if here, no activity reboot the board */
        PNI_PRINTF("[WARNING] Maintenance Mode, no activity timeout, reboot in 2 sec!!!\r\n");
        Clock_Wait(2000);
        HAL_NVIC_SystemReset();
      }

      if (W2ST_CHECK_CONNECTION(W2ST_CONNECT_DFU_TERM))
      {
        gParkState = PARK_STATE_BLE_CONN;
      }
      else
      {
        gParkState = PARK_STATE_BLE_ADVER;
      }

      break;

    case PARK_STATE_UPGRADE:
      // program A2/RTI EEPROM
      break;

    default:
    {
      // should never be here
      PNI_PRINTF("[ERROR] wrong state(%u)\n", gParkState);
    }
  }

}


