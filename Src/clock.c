/**
******************************************************************************
* @file    clock.c 
* @author  Central LAB
* @version V2.1.0
* @date    20-September-2016
* @brief   
******************************************************************************
* @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
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
#include "ble_clock.h"
//#include "TargetFeatures.h"
#include "pni_config.h"
#include "wwdg.h"
//#include "stm32l0xx_ll_usart.h"
//#include "usart.h"
#include "rtc.h"
#include "SENtralA2.h"

const uint32_t CLOCK_SECOND = 1000;

/**
 * @brief  Clock_Init
 * @param  None
 * @retval None
 */
void Clock_Init(void)
{
  // FIXME: as long as Cube HAL is initialized this is OK
  // Cube HAL default is one clock each 1 ms
}

/**
 * @brief  Clock_Time
 * @param  None
 * @retval tClockTime
 */
tClockTime Clock_Time(void)
{
  return HAL_GetTick();
}

/**
 * @brief  Clock_Wait Wait for a multiple of 1 ms.
 * @param  uint32_t i
 * @retval None
 */
void Clock_Wait(uint32_t i)
{
#if ENABLE_WWDG
  while (i > 500)
  {
    if (HAL_WWDG_Refresh(&hwwdg) != HAL_OK)
      Error_Handler();
    HAL_Delay(500);
    i -= 500;
  }

  if (i != 0)
  {
    // last one
    if (HAL_WWDG_Refresh(&hwwdg) != HAL_OK)
      Error_Handler();
    HAL_Delay(i);
  }

  // make sure we exit with fresh restart watchdog
  if (HAL_WWDG_Refresh(&hwwdg) != HAL_OK)
      Error_Handler();
#else
  HAL_Delay(i);
#endif
}


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
