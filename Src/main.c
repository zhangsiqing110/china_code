/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "lptim.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "wwdg.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "adc_monitor.h"
// #include "mpc9808.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_utils.h"
#include "SENtralA2.h"
#include "parking.h"
#include "parking_sensor.h"
//#include "sensor_service.h"
#include "spi_flash.h"
#include "pni_fota.h"
#include "pni_device_info.h"
#include "radio.h"
#include "parking_buffer.h"
#include "parking_fsm.h"
#include "parking_sensor.h"
#include "parking_sensor_rti.h"
#include "parking_module.h"

#if ENABLE_MCP9808
#include "mcp9808.h"
#include "mcp9808_l0xx.h"
#include "park_temp_sns_mcp9808.h"
#endif /* ENABLE_MCP9808 */

#if ENABLE_XDOT_RADIO
#include "xdot.h"
#include "xdot_hex.h"
#include "crypto.h"
#endif /* ENABLE_XDOT_RADIO */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static PNI_ParkingBufferIface m_ParkingBuffer = { 0 };
static PNI_ParkingSensorIface m_ParkingSensor = { 0 };
static PNI_RadioIface m_ParkingRadio = { 0 };

/*
 * Store device hardware info like MAC addresses
 */
PNI_DeviceInfo gPniDeviceInfo = { 0 };

#if ENABLE_XDOT_RADIO
/*
 * PNI LoRa AppEUI for iot-dev
 */
static uint8_t PNI_LORA_STANDARD_APP_EUI[] = {
    0x14, 0x93, 0x46, 0x00, 0x00, 0x00, 0x00, 0x01
};

/*
 * Stringified version of AppEUI for use with xDot AT commands
 */
static char gPniLoraStandardAppEui[sizeof(PNI_LORA_STANDARD_APP_EUI) * 3];

/*
 * PNI LoRa AppKey for iot-dev, generated from hmac_md5(AppEUI, DeviceId)
 */
static uint8_t PNI_LORA_STANDARD_APP_KEY[16] = { 0 };

/*
 * Stringified version of AppKey for use with xDot AT commands
 */
static char gPniLoraStandardAppKey[sizeof(PNI_LORA_STANDARD_APP_KEY) * 3];

static Xdot_Cfg m_XdotCfg = {
  .echo_enabled = 1,
  .ism_band = XDOT_ISM_BAND_US915,
  .freq_sub_band = XDOT_FSB_902_3__903_7,
  .public_network_enabled = 1,
  .network_join_mode = XDOT_NJM_OTA,
  // .network_app_eui = "38:d1:7d:94:5b:bd:97:a5",
  .network_app_eui = NULL,
  .ni_type = XDOT_NI_TYPE_HEX,
  // .network_app_key = "e0:29:75:1d:db:a5:66:de:87:5b:62:fa:2f:79:3c:af",
  .network_app_key = NULL,
  .nk_type = XDOT_NI_TYPE_HEX,
  .sf = XDOT_SF_7_BW_125,
  .acks_enabled = 1,
  .rx_type = XDOT_RX_OUT_TYPE_HEX,
  .txp = 11, /* TX power = 11 dBm */
};

#endif /* ENABLE_XDOT_RADIO */

static const uint64_t RTI_META_EVENT_BITMASK = 4063234762ULL;
static int m_RtiInitComplete = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

static void Init_BlueNRG_Stack(PNI_ParkingVersion *version);
static void Init_BlueNRG_Custom_Services(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#if ENABLE_XDOT_RADIO
int PNI_Radio_Init(PNI_RadioIface *radio, Xdot_Cfg *cfg)
{
    PNI_RadioDeviceId *devid = &gPniDeviceInfo.radio_id;
    int ret;

    ret = pni_radio_xdot_init(radio, &huart1);
    if (ret != PNI_RADIO_RET_OK) {
        return ret;
    }

    /* PPX-70 */
    if ( CHECK_XDOT_FLAG_SET(XDOT_FLAG_PROD_CHANGE_CFG) ) {
      PNI_PRINTF("[RADIO] Change XDot config to EEPROM configuration!!!\r\n");

      /* ism_band is using auto hw detection, leave this part for hw without auto detection */
      if (EEPROM_U32_XDOT_PROD_CFG_ISM_BAND[0] < XDOT_ISM_BAND_MAX) {
        PNI_PRINTF("\tism band: from %u to %u\r\n",
                   cfg->ism_band, EEPROM_U32_XDOT_PROD_CFG_ISM_BAND[0]);

        cfg->ism_band = EEPROM_U32_XDOT_PROD_CFG_ISM_BAND[0];
      }

      if (EEPROM_U32_XDOT_PROD_CFG_SUB_BAND[0] < XDOT_FSB_MAX) {
        PNI_PRINTF("\tsub band: from %u to %u\r\n",
                   cfg->freq_sub_band, EEPROM_U32_XDOT_PROD_CFG_SUB_BAND[0]);

        cfg->freq_sub_band = EEPROM_U32_XDOT_PROD_CFG_SUB_BAND[0];
      }

      if (EEPROM_U32_XDOT_PROD_CFG_SF[0] < XDOT_SF_MAX) {
        PNI_PRINTF("\tsf: from %u to %u\r\n",
                   cfg->sf, EEPROM_U32_XDOT_PROD_CFG_SF[0]);

        cfg->sf = EEPROM_U32_XDOT_PROD_CFG_SF[0];
      }
    }

    ret = radio->configure(radio, cfg);
    if (ret != PNI_RADIO_RET_OK) {
        return ret;
    }

    ret = radio->get_device_id(devid);
    if (ret != PNI_RADIO_RET_OK) {
        return ret;
    }

    PNI_PRINTF("[RADIO] Device ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
            devid->bytes[0], devid->bytes[1], devid->bytes[2], devid->bytes[3],
            devid->bytes[4], devid->bytes[5], devid->bytes[6], devid->bytes[7]);

    // PPC-70: if eeprom production config exist, use eeprom config
    if ( CHECK_XDOT_FLAG_SET(XDOT_FLAG_PROD_EUI) ) {
      memcpy((uint8_t *)PNI_LORA_STANDARD_APP_EUI, EEPROM_U8_XDOT_PROD_EUI,
        sizeof(PNI_LORA_STANDARD_APP_EUI));
    }

    // convert network id byte array to string
    xdot_bytes_to_hex(PNI_LORA_STANDARD_APP_EUI, sizeof(PNI_LORA_STANDARD_APP_EUI),
        gPniLoraStandardAppEui, ':', 0);

    if ( CHECK_XDOT_FLAG_SET(XDOT_FLAG_PROD_NET_KEY) ) {
      memcpy((uint8_t *)PNI_LORA_STANDARD_APP_KEY, EEPROM_U8_XDOT_PROD_NET_KEY,
        sizeof(PNI_LORA_STANDARD_APP_KEY));
    } else {
      // generate network key byte array using hmac_md5(network id, device id)
      hmac_md5(PNI_LORA_STANDARD_APP_EUI, sizeof(PNI_LORA_STANDARD_APP_EUI),
          (uint8_t *)devid, sizeof(*devid), PNI_LORA_STANDARD_APP_KEY);
    }

    // convert network key byte array to string
    xdot_bytes_to_hex(PNI_LORA_STANDARD_APP_KEY, sizeof(PNI_LORA_STANDARD_APP_KEY),
        gPniLoraStandardAppKey, ':', 0);

    // assign string versions to xDot cfg
    cfg->network_app_eui = gPniLoraStandardAppEui;
    cfg->network_app_key = gPniLoraStandardAppKey;

    return PNI_RADIO_RET_OK;
}

#if ENABLE_RADIO_TEST_MODE
int PNI_Radio_Test(PNI_RadioIface *radio, Xdot_Cfg *cfg)
{
  Xdot_TestParams params = {
      .txf = "908600000",
      .txp = 11,
      .msg = "deadbeef",
      .interval_ms = 100,
  };

  /* enable test mode */
  radio->set_test_mode_enabled(radio, 1, &params);

  /* endless loop */
  while (1) {
    Clock_Wait(10);
  }
}
#endif /* ENABLE_RADIO_TEST_MODE */

#endif /* ENABLE_XDOT_RADIO */

static void PNI_Parking_ShowSensorVersion(PNI_ParkingVersion *version)
{
  PNI_PRINTF("+----------------------------------------------+\r\n");
  PNI_PRINTF("| PNI SENtral-A2 FW Version: %u.%u.%u.%u\r\n",
  version->major, version->minor, version->patch, version->build);
  PNI_PRINTF("+----------------------------------------------+\r\n");
}

static void PNI_Parking_ShowHostVersion(PNI_ParkingVersion *version,
    const char *build_date, const char *build_time)
{
  PNI_PRINTF(
        "----------------------------------------------\r\n"
        "ParkingXDot Host - V%.2u.%.2u.%.2u.%.3u!!!\r\n"
        "Compiled %s %s"
      #if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n"
      #elif defined (__GNUC__)
        " (openstm32)\r\n"
      #endif
        "STM32L073 Unique ID: 0x%.08X%.08X%.08X\r\n"
        "----------------------------------------------\r\n"
        , version->major, version->minor, version->patch, version->build,
        build_date, build_time, STM32_UUID[2], STM32_UUID[1], STM32_UUID[0]);
}

static void PNI_Parking_PushLegacyVersions(PNI_ParkingModule *module,
        PNI_ParkingBufferIface *buffer)
{
    PNI_ParkingPacket pkt = {
        .sid = PARK_SNS_TYPE_VERSION,
        .version_info = {
            .host = {
                .major = module->version.host.major,
                .minor = module->version.host.minor,
                .patch = module->version.host.patch,
            },
            .sensor = {
                .major = module->version.sensor.major,
                .minor = module->version.sensor.minor,
                .patch = module->version.sensor.patch,
                .other = 0,
                .build = module->version.sensor.build,
            },
        },
        .timestamp = 0U, /* TODO: get this */
    };

    buffer->push(buffer, &pkt);
}

static void PNI_Parking_PushVersion(PNI_ParkingVersion *version,
        PNI_ParkingBufferIface *buffer)
{
    PNI_ParkingPacket pkt = {
        .sid = PARK_SNS_TYPE_FW_VERSION,
        .version = *version,
        .timestamp = 0U,
    };

    buffer->push(buffer, &pkt);
}

static void PNI_Parking_PushVersions(PNI_ParkingModule *module,
        PNI_ParkingBufferIface *buffer)
{
    /* push legacy version info (host + sensor in one packet) */
    PNI_Parking_PushLegacyVersions(module, buffer);

    /* push new version format (each in own packet) */
    PNI_Parking_PushVersion(&module->version.host, buffer);
    PNI_Parking_PushVersion(&module->version.sensor, buffer);
    PNI_Parking_PushVersion(&module->version.ble, buffer);
}

/* This should go somewhere else ... */
static void PNI_SENtralA2_On_Initialized(void *self, uint16_t ram_ver)
{
  SENtralA2_MetaEvent_Callback *cb = self;
  PNI_ParkingModule *module = cb->params;
  PNI_ParkingSensorIface *park_sns = module->sensor;
  SENtralA2Iface *rti = park_sns->params;

  if (m_RtiInitComplete) {
    PNI_PRINTF("[WARN] already initialized ...\r\n");
    return;
  }

  m_RtiInitComplete = 1;

  /* show version */
  PNI_PRINTF("[INFO] SENtral-A2 initializtion complete!\r\n");
  if (park_sns->get_version(park_sns, &module->version.sensor)
        != PNI_PARKING_RET_OK) {
      PNI_PRINTF("[ERROR] getting parking version\r\n");
  } else {
      PNI_Parking_ShowSensorVersion(&module->version.sensor);
  }
  PNI_Parking_PushVersions(module, module->buffer);

  /* set meta event ctrl */
  PNI_PRINTF("[INFO] setting meta event control ...\r\n");
  if (rti->set_meta_event_ctrl(rti, RTI_META_EVENT_BITMASK,
      RTI_META_EVENT_BITMASK) != SENA2_RET_OK) {
    PNI_PRINTF("[ERROR] setting meta event control\r\n");
  }

  /* enable car detector sensor */
  PNI_PRINTF("[INFO] enabling car detector sensor ...\r\n");
  if (park_sns->set_car_detector_enabled(park_sns, 1) != PNI_PARKING_RET_OK) {
    PNI_PRINTF("[ERROR] enabling car detector sensor\r\n");
  }

}

static void PNI_SENtralA2_Meta_Event_Callback_Init(SENtralA2Iface *rti,
    SENtralA2_MetaEvent_Callback *cb, PNI_ParkingModule *module)
{
  cb->on_initialized = &PNI_SENtralA2_On_Initialized;
  cb->params = module;
  rti->meta_event_cb = cb;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* Define interfaces */
  SENtralA2Iface rti = { 0 };
  SENtralA2_MetaEvent_Callback meta_event_cb = { 0 };
  PNI_ParkingModule park_module = {
    .buffer = &m_ParkingBuffer,
    .sensor = &m_ParkingSensor,
    .radio = &m_ParkingRadio,
    .cfg = { 0 },
    .version = {
        .host = {
            .vid = PARK_VID_HOST,
            .major = (uint8_t)HOST_REL_MAJOR,
            .minor = (uint8_t)HOST_REL_MINOR,
            .patch = (uint8_t)HOST_REL_PATCH,
            .build = (uint32_t)HOST_REL_BUILD,
        },
        .sensor = {
            .vid = PARK_VID_SENSOR,
            .major = 0,
            .minor = 0,
            .patch = 0,
            .build = 0,
         },
        .ble = {
            .vid = PARK_VID_BLE,
            .major = 0,
            .minor = 0,
            .patch = 0,
            .build = 0,
         },
        .lora = {
            .vid = PARK_VID_LORA,
            .major = 0,
            .minor = 0,
            .patch = 0,
            .build = 0,
         },
    },
  };

#if ENABLE_MCP9808
  PNI_ParkingTemperatureSensorIface tmp_sns = { 0 };
  MCP9808_Iface mcp9808 = { 0 };
  MCP9808_L0XXParams mcp9808_params = {
    .hi2c = &MCP9808_I2C_HANDLE,
    .addr = MCP9808_I2C_ADDR,
    .timeout = MCP9808_I2C_TIMEOUT,
  };
#endif /* ENABLE_MCP9808 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_LPTIM1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();

  /* USER CODE BEGIN 2 */
  #if ENABLE_PRINTF
  MX_USART2_UART_Init();
  #endif
  #if ENABLE_WWDG
  MX_WWDG_Init();
  #endif
  #if ENABLE_WAKEUP_TIME
  RTC_CalendarShow();
  #endif

  PNI_Parking_ShowHostVersion(&park_module.version.host, __DATE__, __TIME__);

  #if ENABLE_WWDG
  if (HAL_WWDG_Refresh(&hwwdg) != HAL_OK)
      Error_Handler();
  #endif

  #if ENABLE_XDOT_RADIO
  PNI_PRINTF("\r\n<<<< xDot Radio Init >>>\r\n");
  if (PNI_Radio_Init(park_module.radio, &m_XdotCfg) == PNI_RADIO_RET_OK) {
    PNI_PRINTF("Radio OK!\r\n");
  } else {
    PNI_PRINTF("Rasio FAILED!\r\n");
  }

  #if ENABLE_RADIO_TEST_MODE
    PNI_Radio_Test(&m_ParkingRadio, &m_XdotCfg);
  #endif /* ENABLE_RADIO_TEST_MODE */

  #endif /* ENABLE_XDOT_RADIO */

  #if ENABLE_MCU_ADC
  PNI_PRINTF("\r\nMulti Channel ADC Monitor Init... ");
  /* this if for testing multi channel ADC monitor*/
  BATT_MEANS_PWR_ON();
  Adc_Monitor_Init();
  BATT_MEANS_PWR_OFF();
  PNI_PRINTF("Done\r\n");
  #endif

  #if ENABLE_MCP9808
  /* this is for testing temperature module MPC9808 */
  PNI_PRINTF("\r\nDetecting MPC9808 ... ");
#if 0
  if (Mpc9808_ready() == 1)
  {
    uint8_t id = 0;
    uint8_t revision = 0;
    Mpc9808_GetIdRev(&id, &revision);

    PNI_PRINTF("Found device(id = %u, rev = %u)\r\n", id, revision);
  }
  else
  {
    PNI_PRINTF("Not Found\r\n");
  }
#endif
  /* initialize MCP9808 I2C I/O interface */
  mcp9808_l0xx_init(&mcp9808, &mcp9808_params);

  /* initialize MCP9808 parking temperature sensor implementation */
  park_tmp_sns_mcp9808_init(&tmp_sns, &mcp9808);

  if (tmp_sns.is_ready(&tmp_sns) == PARK_TEMP_SNS_RET_OK) {
    PNI_PRINTF("Found device\r\n");
    park_module.tmp_sns = &tmp_sns;
  } else {
    PNI_PRINTF("Not found\r\n");
  }
  #endif

  #if ENABLE_BLE_FOTA
  PNI_PRINTF("\r\n<<<< START BLE >>>>\r\n");

  /* Initialize all FOTA related software */
  sFLASH_Init();
  Init_BlueNRG_Stack(&park_module.version.ble);
  Init_BlueNRG_Custom_Services();
  PNI_PRINTF("Start BLE Done\r\n");
  #endif

  #if ENABLE_SENTRAL_A2
  PNI_PRINTF("\r\n<<<< RTI module initialize >>>>\r\n");

  // Initialize parking sensor circular buffer
  pni_parking_buffer_init(park_module.buffer);

  // Initialize RTI
  if (SENtralA2_init(&rti) != SENA2_RET_OK) {
    PNI_PRINTF("RTI module init Error\r\n");
  } else {
    PNI_PRINTF("RTI module init Done\r\n");
  }

  // Initialize parking sensor interface
  pni_park_sns_rti_init(park_module.sensor, &rti);
  PNI_SENtralA2_Meta_Event_Callback_Init(&rti, &meta_event_cb, &park_module);

  #endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  #if ENABLE_LED_DBG
    DBG_LED_TOGGLE();
  #endif

  #if ENABLE_WWDG
    if (HAL_WWDG_Refresh(&hwwdg) != HAL_OK)
      Error_Handler();
  #endif

  #if ENABLE_SENTRAL_A2
    Parking_Fsm(&park_module);
  #endif

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure LSE Drive Capability 
    */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */

  // By default - Transmit using USART1
  //HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&ch, 1);

  return ch;
}

/** @brief Initialize the BlueNRG Stack
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Stack(PNI_ParkingVersion *version)
{
  const char BoardName[] = {NAME_PNI_BLE,0};
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;
  uint8_t  hwVersion;
  uint16_t fwVersion;
  uint8_t rdLength = 0;

  PNI_PRINTF("Initializing HCI...");
  /* Initialize the BlueNRG HCI */
  HCI_Init();
  PNI_PRINTF("Done\r\n");

  PNI_PRINTF("Reset BlueNRG hardware...");
  /* Reset BlueNRG hardware */
  BlueNRG_RST();
  PNI_PRINTF("Done\r\n");

  PNI_PRINTF("Get BlueNRG version...");
  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);
  PNI_PRINTF("BlueNRG version: %u, %u\n", hwVersion, fwVersion);
  version->major = (fwVersion >> 8);
  version->minor = (fwVersion >> 4) & 0x0F;
  version->minor = 0;
  version->build = 0;

  PNI_PRINTF("Reset BlueNRG hardware 2nd time...");
  /* 
   * Reset BlueNRG again otherwise it will fail.
   */
  BlueNRG_RST();
  PNI_PRINTF("Done\r\n");

  /* check for manufacture BLE public mac flag */
  if (CHECK_BLE_FLAG_SET(BLE_FLAG_PUBLIC_MAC))
  {
    PNI_PRINTF("[DBG]write config data...");
    memcpy(bdaddr, EEPROM_U8_BLE_PUBLIC_MAC, sizeof(bdaddr));

    PNI_PRINTF("\tpublic address: %.02X-%.02X-%.02X-%.02X-%.02X-%.02X\r\n",
    bdaddr[5], bdaddr[4], bdaddr[3], bdaddr[2], bdaddr[1], bdaddr[0]);
    ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                    CONFIG_DATA_PUBADDR_LEN,
                                    bdaddr);
    if(ret > 0)
    {
        PNI_PRINTF("\r\nSetting Pubblic BD_ADDR failed(0x%.2X)\r\n", ret);
        goto fail;
    }
  }

  PNI_PRINTF("Initializing GATT...");
  ret = aci_gatt_init();    
  if(ret){
     PNI_PRINTF("Failed(%u)\r\n", ret);
     goto fail;
  } else {
    PNI_PRINTF("Done\r\n");
  }

  PNI_PRINTF("Initializing GAP...");
  ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if(ret != BLE_STATUS_SUCCESS){
     PNI_PRINTF("Failed(0x%.2X)\r\n", ret);
     goto fail;
  } else {
    PNI_PRINTF("Done\r\n");
  }

  /* if manufacture didn't config BLE public mac, read the random address */
  if ( CHECK_BLE_FLAG_SET(BLE_FLAG_PUBLIC_MAC) == 0 )
  {
    /* STM DT0070 Design Tip - use random address from radio stack
     * http://www.st.com/content/ccc/resource/technical/document/design_tip/group0/8e/c9/29/21/85/60/4c/7d/DM00321427/files/DM00321427.pdf/jcr:content/translations/en.DM00321427.pdf
     *
     * BLE devices can also use random addresses. From the BlueNRG-MS radio stack version
     * 7.1c and onwards, the random address is generated autonomously by the BlueNRG-MS
     * radio stack upon the first call to the API aci_gap_init(). This address is stored persistently
     * into the BlueNRG-MS Flash. The address value can be read from the application using the
     * tBleStatus aci_hal_read_config_data(uint8_t offset, uint16_t data_len, uint8_t
     * *data_len_out_p, uint8_t *data); command with the parameter offset set equal to 0x80.
     */
    //RTC_CalendarShow();
    PNI_PRINTF("Reading Config Data...\r\n");
    ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS,
              CONFIG_DATA_PUBADDR_LEN, &rdLength, bdaddr);
    if(ret){
       PNI_PRINTF("Failed(%u)\r\n", ret);
       goto fail;
    } else {
       PNI_PRINTF("Done\r\n");
    }
  }

  PNI_PRINTF("Update characteristic...");
  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7/*strlen(BoardName)*/, (uint8_t *)BoardName);
  if(ret){
    PNI_PRINTF("\r\naci_gatt_update_char_value failed(0x%.2X)\r\n", ret);
    while(1);
  } else {
    PNI_PRINTF("Done\r\n");
  }

  PNI_PRINTF("Set Authentication Requirement...");
  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL, 7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING, 123456,
                                     BONDING);
  if (ret != BLE_STATUS_SUCCESS) {
     PNI_PRINTF("\r\nGAP setting Authentication failed(0x%.2X)\r\n", ret);
     goto fail;
  } else {
     PNI_PRINTF("Done\r\n");
  }

  PNI_PRINTF("SERVER: BLE Stack Initialized \r\n"
         "\t\tBoard type=%s HWver=%d, FWver=%d.%d.%c\r\n"
         "\t\tBoardName= %s\r\n"
         "\t\tBoardMAC(%s)= %.02x:%.02x:%.02x:%.02x:%.02x:%.02x\r\n\n",
         "PNI RTI/XDOT Parking Module",
         hwVersion,
         fwVersion>>8,
         (fwVersion>>4)&0xF,
         (hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a',
         BoardName,
        (CHECK_BLE_FLAG_SET(BLE_FLAG_PUBLIC_MAC))? "Public":"Random",
         bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);

  PNI_PRINTF("Set output power level...");
  /* Set output power level */
  aci_hal_set_tx_power_level(1,4);
  PNI_PRINTF("Done\r\n");

  return;

fail:
  return;
}

/** @brief Initialize all the Custom BlueNRG services
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Custom_Services(void)
{
  int ret;
  // PNI FOTA service
#if 0
    ret = Add_old_Dfu_Service();
    if(ret == BLE_STATUS_SUCCESS) {
       PNI_PRINTF("PNI Dfu Service added successfully\r\n");
    } else {
       PNI_PRINTF("\r\nError while adding PNI Dfu Service(0x%.2X)\r\n", ret);
    }
#endif
  // PNI FOTA service
  ret = Add_Dfu_Service();
  if(ret == BLE_STATUS_SUCCESS) {
     PNI_PRINTF("PNI Dfu Service added successfully\r\n");
  } else {
     PNI_PRINTF("\r\nError while adding PNI Dfu Service(0x%.2X)\r\n", ret);
  }

#if 0
  ret = Add_ConsoleW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) {
     PNI_PRINTF("Console Service W2ST added successfully\r\n");
  } else {
     PNI_PRINTF("\r\nError while adding Console Service W2ST(0x%.2X)\r\n", ret);
  }
#endif
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  PNI_PRINTF("[ERROR] Error Handler\r\n");

  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
