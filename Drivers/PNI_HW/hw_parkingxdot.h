/**
* @file         pni_parking_module.h
*
* @brief        PNI RTI/XDot Parking module hardware
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

#ifndef HW_PARKINGXDOT_H
#define HW_PARKINGXDOT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* redefine interface connecting it to blueNRG software */
// SPI Reset Pin: PA.8
#define BNRG_SPI_RESET_PIN                BT_RESET_Pin
#define BNRG_SPI_RESET_MODE               GPIO_MODE_OUTPUT_PP
#define BNRG_SPI_RESET_PULL               GPIO_PULLUP
#define BNRG_SPI_RESET_SPEED              GPIO_SPEED_LOW
#define BNRG_SPI_RESET_ALTERNATE          0
#define BNRG_SPI_RESET_PORT               BT_RESET_GPIO_Port
#define BNRG_SPI_RESET_CLK_ENABLE()       __GPIOA_CLK_ENABLE()

// NSS/CSN/CS: PA.4
#define BNRG_SPI_CS_PIN                   BT_CSN_Pin
#define BNRG_SPI_CS_MODE                  GPIO_MODE_OUTPUT_PP
#define BNRG_SPI_CS_PULL                  GPIO_PULLUP
#define BNRG_SPI_CS_SPEED                 GPIO_SPEED_HIGH
#define BNRG_SPI_CS_ALTERNATE             0
#define BNRG_SPI_CS_PORT                  BT_CSN_GPIO_Port
#define BNRG_SPI_CS_CLK_ENABLE()          __GPIOA_CLK_ENABLE()

// IRQ: PA.1
#define BNRG_SPI_IRQ_PIN                  BT_IRQ_Pin
#define BNRG_SPI_IRQ_MODE                 GPIO_MODE_IT_RISING
#define BNRG_SPI_IRQ_PULL                 GPIO_NOPULL
#define BNRG_SPI_IRQ_SPEED                GPIO_SPEED_HIGH
#define BNRG_SPI_IRQ_ALTERNATE            0
#define BNRG_SPI_IRQ_PORT                 BT_IRQ_GPIO_Port
#define BNRG_SPI_IRQ_CLK_ENABLE()         __GPIOA_CLK_ENABLE()

// EXTI External Interrupt for SPI
// NOTE: if you change the IRQ pin remember to implement a corresponding handler
// function like EXTI0_1_IRQHandler() in the user project
#define BNRG_SPI_EXTI_IRQn                EXTI0_1_IRQn
#define BNRG_SPI_EXTI_IRQHandler          EXTI0_1_IRQHandler
#define BNRG_SPI_EXTI_PIN                 BNRG_SPI_IRQ_PIN
#define BNRG_SPI_EXTI_PORT                BNRG_SPI_IRQ_PORT
#define RTC_WAKEUP_IRQHandler             RTC_IRQHandler

#endif
