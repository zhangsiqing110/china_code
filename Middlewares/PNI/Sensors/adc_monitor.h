/**
* @file         adc_monitor.h
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

#ifndef PNI_ADC_MONITOR_H
#define PNI_ADC_MONITOR_H

#include "main.h"
#include "adc.h"

#define BM_ADC_HANDLER            hadc
#define BM_ADC_CHANNEL            ADC_CHANNEL_8
#define TEMPERATURE_ADC_CHANNEL   ADC_CHANNEL_18

#define BATT_MEANS_PWR_OFF()       HAL_GPIO_WritePin(BATT_MEAS_PWR_GPIO_Port, BATT_MEAS_PWR_Pin, GPIO_PIN_RESET);

#define BATT_MEANS_PWR_ON()      HAL_GPIO_WritePin(BATT_MEAS_PWR_GPIO_Port, BATT_MEAS_PWR_Pin, GPIO_PIN_SET);

#define FACTORY_TSCALIB_MDP_BASE        ((uint32_t)0x1FF80078)    /*!< Calibration Data Bytes base address for medium density plus devices*/
#define FACTORY_TSCALIB_MDP_DATA        ((TSCALIB_TypeDef *) FACTORY_TSCALIB_MDP_BASE)
#define ERROR_READING_TEMPERATURE       255
#define ADC_TEMPERATURE_TIMEOUT         500

typedef struct
{
    uint16_t VREF;
    uint16_t TS_CAL_1; // low temperature calibration data
    uint16_t reserved;
    uint16_t TS_CAL_2; // high temperature calibration data
} TSCALIB_TypeDef;

#define ADC_BATTERY_SCALE         1000.0f


void  Adc_Monitor_Init();
void  Adc_Monitor_Start();
void  Adc_Monitor_Stop();
float  Battery_Monitor_Read();
uint32_t  Temperature_Monitor_Read();
int32_t get_mcu_internl_temperature();


#endif
