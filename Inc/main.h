/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "pni_config.h"
#include "stm32l0xx_hal.h"

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DEBUG_LED_Pin GPIO_PIN_0
#define DEBUG_LED_GPIO_Port GPIOC
#define DEBUG_MODE_Pin GPIO_PIN_1
#define DEBUG_MODE_GPIO_Port GPIOC
#define MEM_MISO_Pin GPIO_PIN_2
#define MEM_MISO_GPIO_Port GPIOC
#define MEM_MOSI_Pin GPIO_PIN_3
#define MEM_MOSI_GPIO_Port GPIOC
#define SENSOR_IRQ_Pin GPIO_PIN_0
#define SENSOR_IRQ_GPIO_Port GPIOA
#define BT_IRQ_Pin GPIO_PIN_1
#define BT_IRQ_GPIO_Port GPIOA
#define AP_DBTX_Pin GPIO_PIN_2
#define AP_DBTX_GPIO_Port GPIOA
#define AP_DBRX_Pin GPIO_PIN_3
#define AP_DBRX_GPIO_Port GPIOA
#define BT_CSN_Pin GPIO_PIN_4
#define BT_CSN_GPIO_Port GPIOA
#define BT_SCK_Pin GPIO_PIN_5
#define BT_SCK_GPIO_Port GPIOA
#define BT_MISO_Pin GPIO_PIN_6
#define BT_MISO_GPIO_Port GPIOA
#define BT_MOSI_Pin GPIO_PIN_7
#define BT_MOSI_GPIO_Port GPIOA
#define BAT_LEVEL_Pin GPIO_PIN_0
#define BAT_LEVEL_GPIO_Port GPIOB
#define TP_1_Pin GPIO_PIN_1
#define TP_1_GPIO_Port GPIOB
#define TP_2_Pin GPIO_PIN_2
#define TP_2_GPIO_Port GPIOB
#define MEM_CSN_Pin GPIO_PIN_12
#define MEM_CSN_GPIO_Port GPIOB
#define MEM_SCK_Pin GPIO_PIN_13
#define MEM_SCK_GPIO_Port GPIOB
#define BT_RESET_Pin GPIO_PIN_8
#define BT_RESET_GPIO_Port GPIOA
#define SENSOR_SCL_Pin GPIO_PIN_9
#define SENSOR_SCL_GPIO_Port GPIOA
#define SENSOR_SDA_Pin GPIO_PIN_10
#define SENSOR_SDA_GPIO_Port GPIOA
#define MEM_PCTRL_Pin GPIO_PIN_11
#define MEM_PCTRL_GPIO_Port GPIOA
#define XDOT_WAKE_Pin GPIO_PIN_12
#define XDOT_WAKE_GPIO_Port GPIOA
#define XDOT_NRST_Pin GPIO_PIN_15
#define XDOT_NRST_GPIO_Port GPIOA
#define TP_3_Pin GPIO_PIN_3
#define TP_3_GPIO_Port GPIOB
#define TP_4_Pin GPIO_PIN_4
#define TP_4_GPIO_Port GPIOB
#define TP_5_Pin GPIO_PIN_5
#define TP_5_GPIO_Port GPIOB
#define XDOT_ATRX_Pin GPIO_PIN_6
#define XDOT_ATRX_GPIO_Port GPIOB
#define XDOT_ATTX_Pin GPIO_PIN_7
#define XDOT_ATTX_GPIO_Port GPIOB
#define BATT_MEAS_PWR_Pin GPIO_PIN_8
#define BATT_MEAS_PWR_GPIO_Port GPIOB
#define PGOOD_Pin GPIO_PIN_9
#define PGOOD_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define STM32_UUID                   ((uint32_t *)0x1FF80050)

#if ENABLE_MCP9808
#define MCP9808_I2C_HANDLE (hi2c1)
#define MCP9808_I2C_ADDR (0x30)
#define MCP9808_I2C_TIMEOUT (1000U)
#endif /* ENABLE_MCP9808 */

/* Debug LED: pin low */
#define DBG_LED_OFF()     HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);
/* Debug LED: pin high */
#define DBG_LED_ON()      HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET);
/* Debug LED: pin Toggle  */
#define DBG_LED_TOGGLE()  HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
