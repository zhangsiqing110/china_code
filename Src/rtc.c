/**
  ******************************************************************************
  * File Name          : RTC.c
  * Description        : This file provides code for the configuration
  *                      of the RTC instances.
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
#include "rtc.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
uint8_t rtcWakeUpFlag = 0;
uint8_t rtcAlarmAFlag = 0;

/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x9;
  sTime.Minutes = 0x7;
  sTime.Seconds = 0x35;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 0x22;
  sDate.Year = 0x17;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0x9;
  sAlarm.AlarmTime.Minutes = 0x8;
  sAlarm.AlarmTime.Seconds = 0x35;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enable the WakeUp 
    */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x2616, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enable the reference Clock input 
    */
  if (HAL_RTCEx_SetRefClock(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_RTC_ENABLE();
  
    /**RTC GPIO Configuration    
    PB15     ------> RTC_REFIN 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_RTC;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn);
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
  
    /**RTC GPIO Configuration    
    PB15     ------> RTC_REFIN 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(RTC_IRQn);

  }
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */
/**
  * @brief  Display the current time and date.
  * @param  showtime : pointer to buffer
  * @param  showdate : pointer to buffer
  * @retval None
  */
void RTC_CalendarShow(void)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  uint8_t showTime[50];
  uint8_t showDate[50];

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

  /* Display time Format : hh:mm:ss */
  sprintf((char*)showTime,"%.2d:%.2d:%.2d", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
  /* Display date Format : mm-dd-yy */
  sprintf((char*)showDate,"%.2d-%.2d-%.2d", sdatestructureget.Month, sdatestructureget.Date, 2000 + sdatestructureget.Year);

  PNI_PRINTF("\r\n--- %s, %s ---\r\n", showDate, showTime);
} 

/**
  * @brief  Configure wakeup timer medium periods (1 sec ~ 18 hours), resolution per second
  * @param  uint16_t wuSeconds
  * @retval None
  */
void RTC_ConfigureWakeUpTimerSec(uint16_t wuSeconds)
{
  uint32_t wakeUpCounter = 0;
  //PNI_PRINTF("[DBG] set wakeup timer %u sec\r\n", wuSeconds);

  wakeUpCounter = wuSeconds;

  /* Disable Wakeup Counter */
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wakeUpCounter, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
}

/**
  * @brief  Configure wakeup timer short periods (122.07 uS ~ 32 sec), resolution per uSec
  * @param  uint16_t wuSeconds
  * @retval None
  */
void RTC_ConfigureWakeUpTimerUSec(uint32_t uSeconds)
{
  uint32_t wakeUpCounter = 0;
  //PNI_PRINTF("[DBG] set wakeup timer %u uSec\r\n", uSeconds);

  /* Disable Wakeup Counter */
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  /*  RTC Wakeup Interrupt Generation:
    Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI))
    Wakeup Time = Wakeup Time Base * WakeUpCounter
    = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI)) * WakeUpCounter
      ==> WakeUpCounter = Wakeup Time / Wakeup Time Base

    To configure the wake up timer to 4s the WakeUpCounter is set to 0x1FFF:
    RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16
    Wakeup Time Base = 16 /(~39.000KHz) = ~0,410 ms
    Wakeup Time = ~4s = 0,410ms  * WakeUpCounter
      ==> WakeUpCounter = ~4s/0,410ms = 9750 = 0x2616 */
  wakeUpCounter = (uSeconds * 1000) / 410;

  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wakeUpCounter, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
}

/**
  * @brief  compare start time to current time and check if the time has expired against numSec
  * @param  RTC_TimeTypeDef startTimeStruct
  *         uint32_t numSec
  * @retval unt8_t Return  (0/1 = no/yes(expired) )
  */
uint8_t RTC_TimeExpired(RTC_TimeTypeDef startTimeStruct, uint32_t numSec)
{
  uint32_t startSecond = 0;
  uint32_t curSecond = 0;
  uint32_t diffSecond = 0;
  uint8_t result = 0;
  RTC_TimeTypeDef stimestructureget;
  RTC_DateTypeDef sdatestructureget;

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

  startSecond = startTimeStruct.Seconds + (startTimeStruct.Minutes * 60) +
                (startTimeStruct.Hours * 3600);

  curSecond = stimestructureget.Seconds + (stimestructureget.Minutes * 60) +
              (stimestructureget.Hours * 3600);

  //PNI_PRINTF("[DBG]start = %u, cur = %u\r\n", startSecond, curSecond);
  if (curSecond == startSecond)
  {
    // should never happen
    diffSecond = 0;
  }
  else if (curSecond > startSecond)
  {
    diffSecond = curSecond - startSecond;
  }
  else
  {
    diffSecond = (curSecond + 86400) - startSecond;
  }

  if (diffSecond >= numSec)
    result = 1;
  else
    result = 0;

  return result;
}

/**
  * @brief  set AlarmA to wake up in numSec
  * @param  uint32_t numSec
  * @retval none
  */
void RTC_SetAlarmA(uint32_t numSec)
{
  uint8_t newHours = 0;
  uint8_t newMins = 0;
  uint8_t newSecs = 0;
  RTC_AlarmTypeDef sAlarm;
  RTC_TimeTypeDef sTimeGet;
  RTC_DateTypeDef sDateGet;

  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

  //Clock_Wait(300);

  /* get current time */
  HAL_RTC_GetTime(&hrtc, &sTimeGet, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDateGet, RTC_FORMAT_BIN);

  /* calculate time to set Alarm */
  newHours = sTimeGet.Hours + (numSec / 3600);
  newMins = sTimeGet.Minutes + ((numSec / 60) % 60);
  newSecs = sTimeGet.Seconds + (numSec % 60);

  /* setup Alarm A
   */
  if (newSecs >= 60)
  {
    newMins++;
    newSecs -= 60;
  }

  if (newMins >= 60)
  {
    newHours++;
    newMins -= 60;
  }

  if (newHours >= 24)
  {
    newHours -= 24;
  }

  sAlarm.AlarmTime.Hours = newHours;
  sAlarm.AlarmTime.Minutes = newMins;
  sAlarm.AlarmTime.Seconds = newSecs;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT_24;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
