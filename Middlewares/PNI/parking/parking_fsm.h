/**
* @file         parking_fsm.h
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

#ifndef PNI_PARKING_FSM_H
#define PNI_PARKING_FSM_H

#include "parking_module.h"

#define FSM_STATUS_OK                     1
#define FSM_STATUS_FAIL                   -1
#define FSM_STATUS_RADIO_NOT_SLEEP        -2

typedef enum pni_park_state {
    PARK_STATE_INIT,
    PARK_STATE_STOP,
    PARK_STATE_DATA_READY,
    PARK_STATE_SLEEP,
    PARK_STATE_GETFIFO,
    PARK_STATE_PARSE,
    PARK_STATE_TX_LORA,

/* MCU won't be stopped - below this point */
    PARK_STATE_MFG,               /* for manufacture mode - enable LORA --> BLE */
    PARK_STATE_MFG_LORA_TEST,     /* manufacture - Lora testing */
    PARK_STATE_MFG_BLE_TEST,      /* manufacture - BLE testing */
    PARK_STATE_MFG_TEST_DONE,     /* reset everything back to default */

    PARK_STATE_MAINT,       /* for maintenance mode - enable BLE */
    PARK_STATE_BLE_ADVER,
    PARK_STATE_BLE_CONN,
    PARK_STATE_SHIP,        /* for Shipping mode - LORA and BLE off  */
    PARK_STATE_UPGRADE,     /* upgrade SENtral-A2 FW */
    PARK_STATE_MAX,
} PNI_ParkingState;

extern uint8_t fsmFlag;

#define FSM_FLAG_BLE_WAKE           (1 << 0)
#define FSM_FLAG_SHIPPING_MODE      (1 << 1)
#define FSM_FLAG_ALARM_0_STOP       (1 << 2)
#define FSM_FLAG_MAG_LOG_START      (1 << 3)

#define CHECK_FSM_FLAG(flag)        ((fsmFlag & (flag))? 1 : 0)
#define SET_FSM_FLAG(flag)          (fsmFlag |= (flag))
#define CLEAR_FSM_FLAG(flag)        (fsmFlag &= (~flag))


void Parking_SetFsm(PNI_ParkingState state);
void Parking_Fsm(PNI_ParkingModule *module);

#endif
