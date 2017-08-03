/**
* @file         pni_fota.h
*
* @brief        PNI BLE FOTA implementation
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

  
/* Define to prevent recursive inclusion -------------------------------------*/  
#ifndef _PNI_OTA_H_
#define _PNI_OTA_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hal_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "gp_timer.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"   
#include "hci.h"
#include "hal.h"
#include "sm.h"
#include "debug.h"

#include <stdlib.h>
#include "main.h"
#include "crc.h"
#include "spi_flash.h"
#include "uuid_ble_service.h"
#include "radio.h"

/* Define the Max dimesion of the Bluetooth characteristics
for each packet used for Dfu Service */
#define PNI_DFU_MAX_CHAR_LEN 20

#define PNI_FOTA_MAGIC_NUM                    0xDEADBEEF
#define PNI_FOTA_INTFLAH_MAGIC_ADDRESS        0x08018000
#define PNI_FOTA_INTFLAH_ADDRESS_START        0x08018008
#define PNI_FOTA_INTFLAH_ADDRESS_END          0x0802FFFF

/*
 * board type: 0x01 - STM32L073 Nucleo
 *           0x02 - RTI/XDot Parking Module
 * board version: 0x01 - Rev A
 *             0x02 - Rev B
 *             0x03 - Rev C
 */
#define BOARD_TYPE_STM_L073                   0x01
#define BOARD_TYPE_PNI_PARKING_XDOT           0x02
#define BOARD_VERSION_REV_A                   0x01
#define BOARD_VERSION_REV_B                   0x02
#define BOARD_VERSION_REV_C                   0x03

//#define PNI_PARKING_HW_INFO                   BOARD_VERSION_REV_C,BOARD_TYPE_STM_L073
#define PNI_PARKING_HW_INFO                   BOARD_VERSION_REV_B,BOARD_TYPE_PNI_PARKING_XDOT
#define PNI_PARKING_SW_INFO                   HOST_REL_MAJOR,HOST_REL_MINOR,HOST_REL_PATCH,HOST_REL_BUILD

/* For enabling the capability to handle BlueNRG Congestion */
#define ACC_BLUENRG_CONGESTION

#ifdef ACC_BLUENRG_CONGESTION
/* For defining how many events skip when there is a congestion */
#define ACC_BLUENRG_CONGESTION_SKIP 30
#endif /* ACC_BLUENRG_CONGESTION */

extern uint32_t ConnectionBleStatus;
extern uint8_t bdaddr[6];
extern uint8_t set_connectable;
extern volatile uint32_t HCI_ProcessEvent;
extern int connected;

/* BLE Characteristic connection control */
/* Environmental Data */
#define W2ST_CONNECT_ENV           (1    )
/* LED status */
#define W2ST_CONNECT_LED           (1<<1 )
/* Acceleration/Gyroscope/Magneto */
#define W2ST_CONNECT_ACC_GYRO_MAG  (1<<2 )

/* DFU Terminal */
#define W2ST_CONNECT_DFU_TERM      (1<<7 )  // TODO: add this in case we need it, check later

/* Standard Terminal */
#define W2ST_CONNECT_STD_TERM      (1<<8 )

/* Standard Error */
#define W2ST_CONNECT_STD_ERR       (1<<9 )

/* HW Advance Features */
#define W2ST_CONNECT_ACC_EVENT     (1<<10)

/* Gas Gouge Feature */
#define W2ST_CONNECT_GG_EVENT      (1<<11)

#define W2ST_CHECK_CONNECTION(BleChar) ((ConnectionBleStatus&(BleChar)) ? 1 : 0)
#define W2ST_ON_CONNECTION(BleChar)    (ConnectionBleStatus|=(BleChar))
#define W2ST_OFF_CONNECTION(BleChar)   (ConnectionBleStatus&=(~BleChar))

// safety feature - if Host enter maintenance mode and no activity for below sec, then the board will reboot
#define OTA_DFU_MAX_NO_ACTIVITY_SEC       300


// OTA DFU related defines
#define OTA_DFU_OK                        1
#define OTA_DFU_FAIL                      -1
#define OTA_DFU_ERR_INCORRECT_FRAME_ID    -2
#define OTA_DFU_ERR_SIZE_OF_FW            -3
#define OTA_DFU_ERR_CHG_BLE_CONN_INTVAL   -4
#define OTA_DFU_ERR_WRITE_FLASH           -5
#define OTA_DFU_ERR_ERASE_FLASH           -6
#define OTA_DFU_ERR_CRC_NOT_MATCH         -7
#define OTA_DFU_ERR_BLE_DATA_ISSUE        -8
#define OTA_DFU_ERR_FOTA_STOPPED          -9
#define OTA_DFU_ERR_INCORRECT_FW_TYPE     -10
#define OTA_DFU_ERR_BLE_UPGRADE           -11
#define OTA_DFU_ERR_RTI_UPGRADE           -12

// using APB2(26MHz) - configure shorter connection interval.
//#define OTA_DFU_BLE_CONN_INTERVAL_MIN     12
//#define OTA_DFU_BLE_CONN_INTERVAL_MAX     12

// using APB2(2.097MHz) - need to configure longer connection interval too.
#define OTA_DFU_BLE_CONN_INTERVAL_MIN     14
#define OTA_DFU_BLE_CONN_INTERVAL_MAX     14



// define Firmware Size for DFU
#define FOTA_DFU_MAX_PROG_SIZE            (0x2C000)   // max: 0x080004000~0x0802ffff(176K)
#define FOTA_DFU_MAX_A2_SIZE              (0x2C000)
#define FOTA_DFU_MAX_BLE_SIZE             (0x10800)   // BLE firmware always = 67584
#define FOTA_DFU_MAX_XDOT_SIZE            (0x2C000)
#define FOTA_DFU_MAX_RTI_SIZE             (0x8000)   // Max A2 firmware = 32768

// define different types of DFU firmware
#define OTA_DFU_FW_TYPE_MCU               0x01
#define OTA_DFU_FW_TYPE_A2                0x02
#define OTA_DFU_FW_TYPE_BLE               0x03
#define OTA_DFU_FW_TYPE_XDOT              0x04
#define OTA_DFU_FW_TYPE_MFG_TEST          0x05


#define OTA_PARKING_DFU_CMD_START         0x01
#define OTA_PARKING_DFU_CMD_DATA          0x02
#define OTA_PARKING_DFU_CMD_STOP          0x03
#define OTA_PARKING_DFU_CMD_ACK           0x04
#define OTA_PARKING_DFU_CMD_RESULT        0x05
#define OTA_PARKING_DFU_CMD_REBOOT        0x06


#define OTA_PARKING_DFU_ACK_OK            0x00
#define OTA_PARKING_DFU_ACK_RETRY         0x01
#define OTA_PARKING_DFU_ACK_ERR           0x02

#define OTA_PARKING_DFU_RESULT_PASS       0x00
#define OTA_PARKING_DFU_RESULT_FAIL       0x01
#define OTA_PARKING_DFU_RESULT_REBOOT     0x02

#define OTA_PARKING_DFU_CMD_SIZE          2
#define OTA_PARKING_DFU_FRAME_ID_SIZE     2
#define OTA_PARKING_DFU_HEADER_SIZE       OTA_PARKING_DFU_CMD_SIZE + OTA_PARKING_DFU_FRAME_ID_SIZE
#define OTA_PARKING_DFU_DATA_SIZE         16

// External SPI flash - memory map
#define PNI_FOTA_MCU_CONFIG_SECTOR            0x00000000
#define PNI_FOTA_EXTFLAH_IMG1_ADDR_START      0x00010000      // MCU IMAGE 1
#define PNI_FOTA_EXTFLAH_IMG2_ADDR_START      0x00040000      // MCU IMAGE 2
#define PNI_FOTA_BLE_CONFIG_SECTOR            0x00070000
#define PNI_FOTA_EXTFLAH_BLE1_ADDR_START      0x00080000      // BLE IMAGE 1
#define PNI_FOTA_EXTFLAH_BLE2_ADDR_START      0x000A0000      // BLE IMAGE 2
#define PNI_FOTA_RTI_CONFIG_SECTOR            0x000C0000
#define PNI_FOTA_EXTFLAH_RTI1_ADDR_START      0x000D0000      // RTI IMAGE 1
#define PNI_FOTA_EXTFLAH_RTI2_ADDR_START      0x000E0000      // RTI IMAGE 2

// configuration sectors - MCU
#define PNI_FOTA_MAGIC_ADDR                   PNI_FOTA_MCU_CONFIG_SECTOR + 0
#define PNI_FOTA_FW_SIZE_ADDR                 0x00000004    // move into image #
#define PNI_FOTA_FW_CRC_ADDR                  0x00000008    // move into image #
#define PNI_FOTA_CUR_OTA_NUM_ADDR             PNI_FOTA_MCU_CONFIG_SECTOR + 0xC
#define PNI_FOTA_STATUS_ADDR                  PNI_FOTA_MCU_CONFIG_SECTOR + 0xD
#define PNI_FOTA_FW_MAX_SIZE                  0x00030000

// configuration sectors - BLE
#define PNI_FOTA_BLE_MAGIC                    PNI_FOTA_BLE_CONFIG_SECTOR + 0
#define PNI_FOTA_BLE_ACTIVE_IMG_NUM           PNI_FOTA_BLE_CONFIG_SECTOR + 4

// configuration sectors - A2/RM3100RTI
#define PNI_FOTA_RTI_ACTIVE_IMG_NUM           PNI_FOTA_RTI_CONFIG_SECTOR + 0


// for OTA status
#define OTA_STATUS_UNSTABLE                   0x1
#define OTA_STATUS_STABLE                     0x2

#define GET_OTA_SIZE_WR_ADDR(addr, otanum) \
    addr = (PNI_FOTA_EXTFLAH_IMG1_ADDR_START + ((otanum - 1) * PNI_FOTA_FW_MAX_SIZE)) + \
           (PNI_FOTA_FW_MAX_SIZE - OTA_PARKING_DFU_DATA_SIZE) + PNI_FOTA_FW_SIZE_ADDR

#define GET_OTA_CRC_WR_ADDR(addr, otanum) \
    addr = (PNI_FOTA_EXTFLAH_IMG1_ADDR_START + ((otanum - 1) * PNI_FOTA_FW_MAX_SIZE)) + \
           (PNI_FOTA_FW_MAX_SIZE - OTA_PARKING_DFU_DATA_SIZE) + PNI_FOTA_FW_CRC_ADDR

#define GET_FOTA_BLE_sFLASH_START_ADDR(addr, otanum) \
    addr = PNI_FOTA_EXTFLAH_BLE1_ADDR_START + ((otanum -1) * 0x20000)

#define GET_FOTA_BLE_sFLASH_FRAME_ADDR(addr, otanum, curFrame) \
      addr = (PNI_FOTA_EXTFLAH_BLE1_ADDR_START + ((otanum -1) * 0x20000)) + \
             ((curFrame - 1) * OTA_PARKING_DFU_DATA_SIZE)

#define GET_FOTA_RTI_sFLASH_START_ADDR(addr, otanum) \
    addr = PNI_FOTA_EXTFLAH_RTI1_ADDR_START + ((otanum -1) * 0x10000)

#define GET_FOTA_RTI_sFLASH_FRAME_ADDR(addr, otanum, curFrame) \
    addr = (PNI_FOTA_EXTFLAH_RTI1_ADDR_START + ((otanum -1) * 0x10000)) + \
           ((curFrame - 1) * OTA_PARKING_DFU_DATA_SIZE)

// PNI OTA DFU functions
extern int8_t OtaDfuUpdateMcuOtaImgNum();
extern void       setConnectable(PNI_RadioDeviceId *radio_id);
extern void       HCI_Event_CB(void *pckt);
extern tBleStatus Add_Dfu_Service(void);

#ifdef __cplusplus
}
#endif

#endif /* _PNI_OTA_H_ */
