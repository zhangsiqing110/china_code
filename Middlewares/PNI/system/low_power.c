/**
* @file			low_power.h
*
* @brief		pni low power control
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
#include "stm32l0xx_hal.h"
#include "low_power.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static u32 lowPowerState = 0;

/* Private function prototypes -----------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/

void low_power_disable()
{
  lowPowerState |= e_LOW_POWER_SENTRAL;
}

void low_power_enable()
{
  lowPowerState &= ~e_LOW_POWER_SENTRAL;
}

u32 low_power_get_state( void )
{
  return lowPowerState;
}

void low_power_handler( void )
{
#if 1
  if (lowPowerState == e_LOW_POWER_DEEPSLEEP)
  {
    /* Enter Stop Mode */
    //PNI_PRINTF("Jobs done, time to deep sleep!!\n");
    PNI_PRINTF("s");

    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    //PNI_PRINTF("who wake me up?\n");

    /* Configures system clock after wake-up from STOP */
    SystemClock_Config();
    //PNI_PRINTF("I am ready for work!!\n");
    PNI_PRINTF("S");

  }
#endif
#if 0

  {
    /* Enter Sleep Mode */
    //PNI_PRINTF("Job done, time to sleep!!\n");

    /*Suspend Tick increment to prevent wakeup by Systick interrupt.
      Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)*/
    HAL_SuspendTick();

    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

    //PNI_PRINTF("who wake me up?\n");
    //PNI_PRINTF("I am ready for work!!\n");
  }
#endif
}
