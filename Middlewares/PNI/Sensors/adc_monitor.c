/**
* @file         adc_monitor.c
*
* @brief        PNI STM32L073 ADC monitor funtions 
*
* @date         02/03/2017
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

#include "adc_monitor.h"
#include <string.h>

TSCALIB_TypeDef calibdata;    /* field storing temp sensor calibration data */

/* Variable used to get converted value */
__IO float uwADCBattConvertedValue = 0;
__IO uint32_t uwADCTSConvertedValue = 0;
__IO uint32_t uwADCVRefConvertedValue = 0;

__IO uint32_t adcRaw[3];
__IO uint32_t index = 0;

__IO uint32_t rawSaved = 0;

//__IO uint32_t error = 0;
//__IO uint32_t state = 0;


int32_t ComputeTemperature(uint32_t measure)
{
 int32_t temperature;

 /* Ruairi - PEK-30 -offset by -3 celsius vs MPC9808*/
 memcpy(&calibdata, FACTORY_TSCALIB_MDP_DATA, sizeof(calibdata));
 temperature =(int32_t)((100.0 /((int32_t)calibdata.TS_CAL_2 - (int32_t)calibdata.TS_CAL_1) * ((int32_t)measure - (int32_t)calibdata.TS_CAL_1)) + 30.0);

 return(temperature);
}

int32_t get_mcu_internl_temperature(){
  static int temp;
  memcpy(&calibdata, FACTORY_TSCALIB_MDP_DATA, sizeof(calibdata));

  if (HAL_ADC_Start(&BM_ADC_HANDLER) != HAL_OK && (HAL_ADC_PollForConversion(&BM_ADC_HANDLER, ADC_TEMPERATURE_TIMEOUT) != HAL_OK))
  {
    PNI_PRINTF("Error readin internal temperature!");
    HAL_ADC_Stop(&BM_ADC_HANDLER);
    return ERROR_READING_TEMPERATURE;
  }
  else
  {
   adcRaw[0] = BM_ADC_HANDLER.Instance->DR;
   adcRaw[1] = BM_ADC_HANDLER.Instance->DR;
   adcRaw[2] = BM_ADC_HANDLER.Instance->DR;
   //PNI_PRINTF("[DBG]130 cal = %u, 30 cal = %u\r\n", calibdata.TS_CAL_2, calibdata.TS_CAL_1 );

    temp = (int)((100.0 /((int32_t)calibdata.TS_CAL_2 - (int32_t)calibdata.TS_CAL_1) * ((int32_t)adcRaw[1] - (int32_t)calibdata.TS_CAL_1)) + 30.0);
    HAL_ADC_Stop(&BM_ADC_HANDLER);
  }

  return temp;
}

float ComputeBattery(uint32_t measure)
{
  float battVoltage;

  battVoltage = (measure * 3.65f) / 4095.0f;
  return battVoltage;
}


/**
  * @brief  callback with converted battery monitor value.
  * @param  AdcHandler
  * @note
  * @retval None
  */
void  HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
  if (__HAL_ADC_GET_FLAG(AdcHandle, ADC_FLAG_EOC))
  {
    adcRaw[index] = HAL_ADC_GetValue(AdcHandle);
    index++;
  }
   
  if (__HAL_ADC_GET_FLAG(AdcHandle, ADC_FLAG_EOS))
  {
    // last interrupt will be EOS
    adcRaw[index] = HAL_ADC_GetValue(AdcHandle);

    index = 0;
    //rawSaved = adcRaw[0];
    //rawSaved = adcRaw[1];

    uwADCBattConvertedValue = ComputeBattery(adcRaw[0]);
    uwADCTSConvertedValue = ComputeTemperature(adcRaw[1]);
    uwADCVRefConvertedValue = /*3300 * (*VREFINT_CAL_ADDR) / */adcRaw[2];
    
#if 1
    /* ### Stop conversion in Interrupt mode ########################### */
    if (HAL_ADC_Stop_IT(&BM_ADC_HANDLER) != HAL_OK)
    {
      Error_Handler();
    }
#endif
  }   

}

/**
  * @brief  Initialize the ADC monitor unit
  * @param  None
  * @note
  * @retval None
  */
void  Adc_Monitor_Init()
{
  /* ### Start calibration ############################################ */
  if (HAL_ADCEx_Calibration_Start(&BM_ADC_HANDLER, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  start ADC monitor
  * @param  None
  * @note
  * @retval None
  */
void  Adc_Monitor_Start()
{
  uwADCBattConvertedValue = 0;
  uwADCTSConvertedValue = 0;
  uwADCVRefConvertedValue = 0;

  /* ### Start conversion in Interrupt mode ########################### */
  if (HAL_ADC_Start_IT(&BM_ADC_HANDLER) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  stop ADC monitor
  * @param  None
  * @note
  * @retval None
  */
void  Adc_Monitor_Stop()
{
  /* ### Stop conversion in Interrupt mode ########################### */
  if (HAL_ADC_Stop_IT(&BM_ADC_HANDLER) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Read the battery monitor value
  * @param  None
  * @note
  * @retval saved battery converted value from ADC
  */
float Battery_Monitor_Read()
{
  //PNI_PRINTF("[DBG]saved Raw measurement = %u\r\n", rawSaved);
  return uwADCBattConvertedValue;
}

/**
  * @brief  Read the saved temperature value. The temperature is a junction temperature for the MCU. 
  * @param  None
  * @note
  * @retval saved temperature value
  */
uint32_t  Temperature_Monitor_Read()
{
  //PNI_PRINTF("[DBG]saved Raw measurement = %u\r\n", rawSaved);
  //PNI_PRINTF("[DBG]130 cal = %u, 30 cal = %u\r\n", *TEMP130_CAL_ADDR, *TEMP30_CAL_ADDR );

  return uwADCTSConvertedValue;
}

