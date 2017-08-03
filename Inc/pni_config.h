/**
* @file         pni_conf.h
*
* @brief        PNI configurations
*
* @date         12/08/2016
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

#ifndef PNI_CONF_H
#define PNI_CONF_H

#include "autogen_version.h"

/*************** Debug Defines ******************/

#define NAME_PNI_BLE 'P','N','I','-','D','F','U'  /* BLE display name */

/* Select configurations */
#define PARKING_XDOT_CONFIG       1         /* production */
#define ENG_TEST_CONFIG_1         0         /* testing config - BLE enabled */
#define XDOT_FIELD_TEST_CONFIG    0         /* for field testing */


/*****************************************************
 *  PARKING XDOT CONFIG                                    *
 *****************************************************/
#if PARKING_XDOT_CONFIG
#define ENABLE_SENTRAL_A2         1
#define ENABLE_BLE_FOTA           1
#define ENABLE_WWDG               1
#define ENABLE_MCU_ADC            1
#define ENABLE_MCP9808            1
#define ENABLE_MCU_STOP_MODE      1         // 0: MCU won't go into Stop
#define ENABLE_XDOT_RADIO         1
#define ENABLE_XDOT_SLEEP         1
#define ENABLE_RTC_ALARM          1

/**** feature under development ****/

/**** testing code/Debug - should be disable in final product ****/
#define ENABLE_ADC_TESTCODE       0
#define ENABLE_LED_DBG            0
#define ENABLE_PRINTF             1     // enable printf
#define ENABLE_WAKEUP_TIME        0     // not display rtc time on console
#define ENABLE_RADIO_TEST_MODE    0
#define TEST_BLE_FOTA_STATE       0
#endif


/*****************************************************
 *  ENGINEER TEST CONFIG 1                                 *
 *****************************************************/
#if ENG_TEST_CONFIG_1
#define ENABLE_SENTRAL_A2         1
#define ENABLE_BLE_FOTA           1
#define ENABLE_WWDG               1
#define ENABLE_MCU_ADC            1
#define ENABLE_MCP9808            1
#define ENABLE_MCU_STOP_MODE      1         // 0: MCU won't go into Stop
#define ENABLE_XDOT_RADIO         1
#define ENABLE_XDOT_SLEEP         1
#define ENABLE_RTC_ALARM          1

/**** feature under development ****/

/**** testing code/Debug - should be disable in final product ****/
#define ENABLE_ADC_TESTCODE       1
#define ENABLE_LED_DBG            1
#define ENABLE_PRINTF             1
#define ENABLE_WAKEUP_TIME        1

#define TEST_BLE_FOTA_STATE       0       // enable this will cause device enter maintenance mode after A2 events
#endif


/*****************************************************
 *  XDOT_FIELD_TEST_CONFIG                                  *
 *****************************************************/
#if XDOT_FIELD_TEST_CONFIG
#define ENABLE_SENTRAL_A2         1
#define ENABLE_BLE_FOTA           1
#define ENABLE_WWDG               1
#define ENABLE_MCU_ADC            1
#define ENABLE_MCP9808            1
#define ENABLE_MCU_STOP_MODE      1         // 0: MCU won't go into Stop
#define ENABLE_XDOT_RADIO         1
#define ENABLE_XDOT_SLEEP         1
#define ENABLE_RTC_ALARM          1

/**** feature under development ****/

/**** testing code/Debug - should be disable in final product ****/
#define ENABLE_ADC_TESTCODE       1
#define ENABLE_LED_DBG            0
#define ENABLE_PRINTF             1
#define ENABLE_WAKEUP_TIME        1

#define TEST_BLE_FOTA_STATE       0       // enable this will cause device enter maintenance mode after A2 events
#endif

/* for enabling the printf on UART2 */
#if ENABLE_PRINTF
#define PNI_PRINTF(...)     printf(__VA_ARGS__)
#else
#define PNI_PRINTF(...)
#endif

#endif  // PNI_CONF_H
