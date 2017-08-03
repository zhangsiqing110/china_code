/**
* @file         pni_fota.c
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

#include <stdio.h>
#include "pni_fota.h"
#include "bluenrg_utils.h"
#include "bluenrg_l2cap_aci.h"
#include "SENtralA2.h"
#include "ProgramEEPROM.h"
#include "parking_fsm.h"

#undef DBG_DELAY
#ifdef DBG_DELAY
static uint32_t endTime;
static uint32_t startTime;
#endif

//#define FOTA_TESTING
#ifdef FOTA_TESTING
#define FOTA_TEST_4                   0
#define FOTA_TEST_6                   1
#endif
/* Imported variables ---------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
int connected = FALSE;
uint8_t set_connectable = FALSE;
uint8_t bdaddr[6];
uint32_t ConnectionBleStatus = 0;

/* Local defines -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static uint8_t BufferToWrite[256];
static int32_t BytesToWrite;

static uint32_t SizeOfUpdateBlueFW=0;
//static uint32_t AspecteduwCRCValue=0;

static uint16_t totalFwFrames=0;
static uint16_t curFrameId = 0;

static uint32_t fwDataCrc = 0;
static uint8_t  otaFwNum = 0;

static uint16_t fwType=0;

#if FOTA_TEST_4 || FOTA_TEST_6
static uint8_t only_one_time = 0;
#endif

// for manufacture testing
static uint32_t totalFotaTime = 0;
static uint8_t mfgTestResult = 0;

#define MFG_TEST_PASS         1
#define MFG_TEST_FAIL         2
#define MFG_FW_SIZE           0x100

static uint16_t connection_handle = 0;

// for PNI OTA DFU

static uint16_t DfuServHandle;
static uint16_t dfuRxPktCharHandle;
static uint16_t dfuTxPktCharHandle;
static uint8_t LastDfuBuffer[PNI_DFU_MAX_CHAR_LEN];
volatile uint32_t HCI_ProcessEvent=0;

/* Local function prototypes --------------------------------------------------*/
#ifdef ACC_BLUENRG_CONGESTION
#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
static int32_t breath;


/* @brief  Update the value of a characteristic avoiding (for a short time) to
 *         send the next updates if an error in the previous sending has
 *         occurred.
 * @param  servHandle The handle of the service
 * @param  charHandle The handle of the characteristic
 * @param  charValOffset The offset of the characteristic
 * @param  charValueLen The length of the characteristic
 * @param  charValue The pointer to the characteristic
 * @retval tBleStatus Status
 */
tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle,
              uint16_t charHandle,
              uint8_t charValOffset,
              uint8_t charValueLen,
              const uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;

  if (breath > 0) {
    breath--;
  } else {
    ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);

    if (ret != BLE_STATUS_SUCCESS){
      breath = ACC_BLUENRG_CONGESTION_SKIP;
    }
  }

  return (ret);
}

#else /* ACC_BLUENRG_CONGESTION */
#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t addr[6] Address of peer device
 * @param  uint16_t handle Connection handle
 * @retval None
 */
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{
  connected = TRUE;
  connection_handle = handle;

  PNI_PRINTF(">>>>>>CONNECTED %x:%x:%x:%x:%x:%x\r\n",addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);

  ConnectionBleStatus=0;
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None
 * @retval None
 */
static void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;

  PNI_PRINTF("<<<<<<DISCONNECTED\r\n");

  /* Make the device connectable again. */
  set_connectable = TRUE;

  ConnectionBleStatus=0;
}

/**
 * @brief  Send DFU Ack Message
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
static tBleStatus Dfu_Ack(uint8_t *data,uint8_t length)
{
  //PNI_PRINTF("[DBG] Dfu_Ack()\r\n");
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages */
  for(Offset =0; Offset<length; Offset +=PNI_DFU_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>PNI_DFU_MAX_CHAR_LEN) ?  PNI_DFU_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastDfuBuffer,data+Offset,DataToSend);
    //LastDfuLen = DataToSend;

    //ret = aci_gatt_update_char_value(DfuServHandle, DfuCtrlCharHandle, 0, DataToSend , data+Offset);
    ret = aci_gatt_update_char_value(DfuServHandle, dfuTxPktCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
      PNI_PRINTF("Error Updating Stdout Char\r\n");
      return BLE_STATUS_ERROR;
    }
    HAL_Delay(20);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Change BLE min/max connection Interval, slave latency always 0, and timeout always 400
 * @param uint16_t minimal interval
 * @param uint16_t maximum interval
 * @retval tBleStatus Status
 */
static tBleStatus Change_BLE_Min_Max_Conn_Interval(uint16_t min, uint16_t max)
{
  /* Reduce the connection interval */
  int ret = aci_l2cap_connection_parameter_update_request(connection_handle,
                min /* interval_min*/,
                max /* interval_max */,
                0   /* slave_latency */,
                400 /*timeout_multiplier*/);

  /* if there is error */
  if (ret != BLE_STATUS_SUCCESS)
  {
    // TODO: do we need to do something?
    PNI_PRINTF("Problem Changing the connection interval, %u\r\n", ret);
  }

  return ret;
}

static int8_t OtaDfuRestoreRtiFw()
{
  uint8_t failFwNum = 0;
  int8_t  result = 0;

  sFLASH_ReadBuffer(&failFwNum, PNI_FOTA_RTI_ACTIVE_IMG_NUM, 1);

  /* restore the active image */
  uint8_t switchOtaNum = (failFwNum == 1)? 2 : 1;
  PNI_PRINTF("[DBG] restore active sector(%u)...\r\n", switchOtaNum);
  sFLASH_EraseSector(PNI_FOTA_RTI_CONFIG_SECTOR);
  sFLASH_WritePage(&switchOtaNum, PNI_FOTA_RTI_ACTIVE_IMG_NUM, 1);

  /* restore EEPROM */
  if (eeprom_same_firmware())
  {
    PNI_PRINTF("[ERROR] something wrong with previous stable image(%u)\r\n", switchOtaNum);
    /* nothing we can do, if previous image also have problem, try erase eeprom and reboot */
    eeprom_erase();

    PNI_PRINTF("\r\n\r\nRebooting system in 3 sec...\r\n");
    /* reset the board in 3 sec */
    Clock_Wait(3000);
    HAL_NVIC_SystemReset();
  }
  else
  {
    // start programming
    if (ProgramEEPROM())
    {
      // reset/restart sentral, after programming
      SentralWrite(RESET_REQ_REG,0x01);

      //Read Sensor info
      displayDeviceIdRegisters();
      SentralWrite(CHIP_CONTROL_REG, 1);
      displayA2StatusRegisters();

      Clock_Wait(1000);
      result = OTA_DFU_OK;
    }
    else
    {
      PNI_PRINTF("[ERROR] Unable to restore RTI firmware\r\n");
      result = OTA_DFU_ERR_RTI_UPGRADE;
    }
  }

  return result;
}

/**
 * @brief Function for upgrade RTI module firmware
 * @param
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuUpgradeRtiFw()
{
  int8_t result = 0;
  /*
   * check EEPROM SENtral image, if different then update the firmware
   */
  if (eeprom_same_firmware() == FALSE)
  {
    // start programming
    if (ProgramEEPROM())
    {
      // reset/restart sentral, after programming
      SentralWrite(RESET_REQ_REG,0x01);

      //Read Sensor info
      displayDeviceIdRegisters();
      SentralWrite(CHIP_CONTROL_REG, 1);
      displayA2StatusRegisters();

      Clock_Wait(1000);
      result = OTA_DFU_OK;
    }
    else
    {
      PNI_PRINTF("[ERROR]Program RM3100 RTI image, restore previous firmware\r\n");
      OtaDfuRestoreRtiFw();
      result = OTA_DFU_ERR_RTI_UPGRADE;
    }
  }

  return result;
}

/**
 * @brief Function for upgrade BLE stack
 * @param uint32_t fw_size size of BLE firmware
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuUpgradeBleStack(uint32_t fw_size)
{
  int ret = 0;
  uint32_t bleImgAddr = 0;
  GET_FOTA_BLE_sFLASH_START_ADDR(bleImgAddr, otaFwNum);
  ret = pni_program_device(bleImgAddr, fw_size);

  if (ret != 0) {
    PNI_PRINTF("OtaDfuUpdateBleStack() - Failure(%u)\r\n", ret);

    PNI_PRINTF("<<<< Restore BLE stack >>>>\r\n");

    // switching back to previous image
    otaFwNum = (otaFwNum == 1)? 2 : 1;
    GET_FOTA_BLE_sFLASH_START_ADDR(bleImgAddr, otaFwNum);

    // we have to assume switching back to previous BLE image will alway work
    pni_program_device(bleImgAddr, fw_size);
  }
  ret = OTA_DFU_OK;

  return ret;
}

static uint8_t OtaDfuCheckMfgData(uint16_t frameId,uint8_t * mfgData, uint8_t dataSize)
{
  uint8_t cmpResult = MFG_TEST_PASS;
  uint8_t testByte;
  uint8_t cmpData[MFG_FW_SIZE] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
    0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B,
    0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x44, 0x4F, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x55,
    0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B,
    0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77,
    0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F, 0x80, 0x81, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x88, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
    0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B,
    0x9C, 0x99, 0x9E, 0x9F, 0xA0, 0xA1, 0xA2, 0xA3, 0xAA, 0xA5, 0xA6, 0xA7,
    0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB0, 0xB1, 0xB2, 0xB3,
    0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
    0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB,
    0xCC, 0xCD, 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7,
    0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF, 0xE0, 0xE1, 0xE2, 0xE3,
    0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF,
    0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB,
    0xFC, 0xFD, 0xFE, 0xFF
  };

  if(frameId == 1) PNI_PRINTF("[DBG] Comparing");
  for (int i = 0; i < dataSize; i++)
  {
    testByte = ((frameId - 1) * OTA_PARKING_DFU_DATA_SIZE) + i;
    //PNI_PRINTF(" - 0x%.02X vs 0x%.02X\r\n", frameId, cmpData[testByte], mfgData[i]);
    PNI_PRINTF(".");
    if (cmpData[testByte] != mfgData[i])
    {
      cmpResult = MFG_TEST_FAIL;
      break;
    }
  }
  if(frameId == totalFwFrames) PNI_PRINTF("Done\r\n");

  return cmpResult;
}


static int8_t OtaDfuCheckCrc8(uint8_t crc, uint8_t * data, uint8_t dataSize)
{
  int8_t returnValue = 0;
  uint8_t crcCal = 0;

  //PNI_PRINTF("crc received = 0x%.02X\r\n", crc);
  for (int i = 0; i < dataSize; i++)
  {
    crcCal += data[i];
    //PNI_PRINTF("crc(0x%.02X) = %d(0x%X)\r\n", data[i], crcCal, crcCal);
  }

  #if FOTA_TEST_6
  if (only_one_time++ == 100)
    crcCal = 0;
  #endif

  if (crcCal != crc)
  {
    PNI_PRINTF("[ERROR] incorrect crc8, (L073crc = 0x%.02X vs crc = 0x%.02X)\r\n",
        crcCal, crc);
    returnValue = OTA_DFU_ERR_BLE_DATA_ISSUE;
  }
  else
  {
    returnValue = OTA_DFU_OK;
  }
  return returnValue;
}

/**
 * @brief Function for write configurations sector 0
 * @param uint32_t address to wirte Magic number
 * @param uint32_t magic number
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuConfigSector()
{
  int8_t returnValue = 0;
  uint32_t address = 0;
  uint32_t magicNum = 0;

  if (fwType == OTA_DFU_FW_TYPE_MCU)
  {
    magicNum = PNI_FOTA_MAGIC_NUM;
    uint8_t *PointerByte = (uint8_t *) &magicNum;
    sFLASH_WritePage(PointerByte, PNI_FOTA_MAGIC_ADDR, 4);

    PointerByte = (uint8_t *) &SizeOfUpdateBlueFW;
    GET_OTA_SIZE_WR_ADDR(address, otaFwNum);
    sFLASH_WritePage(PointerByte, address, 4);

    PointerByte = (uint8_t *) &fwDataCrc;
    GET_OTA_CRC_WR_ADDR(address, otaFwNum);
    sFLASH_WritePage(PointerByte, address, 4);
    //PNI_PRINTF("[DBG] Magic Number-0x%.08X[0x%.02X%.02X%.02X%.02X]\r\n",
    //    magicNum, PointerByte[3],PointerByte[2],PointerByte[1],PointerByte[0]);
    returnValue = OTA_DFU_OK;
  }
  else if(fwType == OTA_DFU_FW_TYPE_A2)
  {
    PNI_PRINTF("[DBG] Writing A2/RTI configuration sector...\r\n");
    //uint8_t switchOtaNum = (otaFwNum == 1)? 2 : 1;
    sFLASH_WritePage(&otaFwNum, PNI_FOTA_RTI_ACTIVE_IMG_NUM, 1);
    returnValue = OTA_DFU_OK;
  }
  else if(fwType == OTA_DFU_FW_TYPE_BLE)
  {
    PNI_PRINTF("[DBG] Writing BLE configuration sector...\r\n");
    //uint8_t switchOtaNum = (otaFwNum == 1)? 2 : 1;
    sFLASH_WritePage(&otaFwNum, PNI_FOTA_BLE_ACTIVE_IMG_NUM, 1);
    returnValue = OTA_DFU_OK;
  }
  else
  {
    PNI_PRINTF("[ERROR] wrong firmware type\r\n");
    returnValue = OTA_DFU_ERR_INCORRECT_FW_TYPE;
  }

  return returnValue;
}

/**
 * @brief Function for write firmware to OTA
 * @param none
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuWriteFlash(uint32_t Address, uint8_t * data, uint8_t data_length)
{
  int8_t returnValue = 0;

  sFLASH_WritePage(data, Address, data_length);

  return returnValue;
}

/**
 * @brief Function for erasing flash location to store RTI/A2 image
 * @param uint8_t imageNum Erase BLE image number
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuEraseRtiSectors(uint8_t imageNum)
{
  int8_t returnValue = 0;
  uint32_t startAddr = 0;

  // erase sections 12 - for configurations
  sFLASH_EraseSector(PNI_FOTA_RTI_CONFIG_SECTOR);

  // select right sector to erase
  if (imageNum == 1)
    startAddr = PNI_FOTA_EXTFLAH_RTI1_ADDR_START;
  else
    startAddr = PNI_FOTA_EXTFLAH_RTI2_ADDR_START;

  sFLASH_EraseSector(startAddr);

  return returnValue;
}

/**
 * @brief Function for erasing flash location to store BLE
 * @param uint8_t imageNum Erase BLE image number
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuEraseBleSectors(uint8_t imageNum)
{
  int8_t returnValue = 0;
  uint32_t startAddr = 0;

  // erase sections 10 - for configurations
  sFLASH_EraseSector(PNI_FOTA_BLE_CONFIG_SECTOR);

  // erase sectors based on image number
  startAddr = PNI_FOTA_EXTFLAH_BLE1_ADDR_START + ((imageNum - 1) * 0x20000);
  sFLASH_EraseSector(startAddr);
  sFLASH_EraseSector(startAddr+0x10000);

  return returnValue;
}

/**
 * @brief Function for erase flash sectors to store Mcu image
 * @param none
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuEraseMcuSectors()
{
  // PNI_PRINTF("[DBG]OtaDfuPrepareFlash()\r\n");
  int8_t returnValue = 0;
  //uint32_t addr = 0;

  // find out which ota image to use
  sFLASH_ReadBuffer(&otaFwNum, PNI_FOTA_CUR_OTA_NUM_ADDR, 1);

  // erase sections 0 - for configurations
  sFLASH_EraseSector(0x00000000);

  // If we install application on new flash
  if ((otaFwNum != 1) && (otaFwNum != 2))
  {
    PNI_PRINTF("[DBG] Should only happen 1 time. OTA firmware Num(%u)\r\n", otaFwNum);
    // upload ota firmware to number 1
    otaFwNum = 1;
  }

  //PNI_PRINTF("[DBG]OTA firmware upload to image %u\r\n", otaFwNum);

  // erase 3 sectors: 1~3/4~6
  uint32_t startAddr = PNI_FOTA_EXTFLAH_IMG1_ADDR_START + ((otaFwNum -1) * PNI_FOTA_FW_MAX_SIZE);
  sFLASH_EraseSector(startAddr);
  sFLASH_EraseSector(startAddr+0x10000);
  sFLASH_EraseSector(startAddr+0x20000);

  // restore the ota image num
  sFLASH_WriteBuffer(&otaFwNum, PNI_FOTA_CUR_OTA_NUM_ADDR, 1);

  // restore boot status
  uint8_t bootStatus = OTA_STATUS_STABLE;
  sFLASH_WriteBuffer(&bootStatus, PNI_FOTA_STATUS_ADDR, 1);

  returnValue = OTA_DFU_OK;

  #if 0 /* enable this to check flash erased value */
  uint8_t dbgBuf[16];
  sFLASH_ReadBuffer(dbgBuf, PNI_FOTA_EXTFLAH_ADDRESS_START, OTA_PARKING_DFU_DATA_SIZE);
  PNI_PRINTF("[0x%.08X:]", PNI_FOTA_EXTFLAH_ADDRESS_START);

  //address = address + 16;
  for (int i = 0; i < 16; i++)
  {
    PNI_PRINTF("0x%.02X ", dbgBuf[i]);
  }
  PNI_PRINTF("\r\n");
  #endif

  return returnValue;
}

/**
 * @brief Function for getting ota image number to use
 * @param uint32_t external flash address store active Image
 * @retval int8_t Return value of the ota image num(opposite of Active image number)
 */
static uint8_t OtaDfuGetOtaImgNum(uint32_t activeImgAddr)
{
  // find out which otaFwNum to use
  uint8_t rdActiveImg = 0;
  uint8_t otaImgNum = 0;
  sFLASH_ReadBuffer(&rdActiveImg, activeImgAddr, 1);
  PNI_PRINTF("[DBG]OtaDfuPrepareFlash(), activeImage(%u)\r\n", rdActiveImg);
  if ((rdActiveImg != 1) && (rdActiveImg != 2))
  {
    otaImgNum = 1;
  }
  else
  {
    otaImgNum = (rdActiveImg == 1)? 2 : 1;
  }
  return  otaImgNum;
}

/**
 * @brief Function for prepare flash ready for OTA
 * @param uint16_t OTA firmware type
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuPrepareFlash(uint16_t fwType)
{
  // PNI_PRINTF("[DBG]OtaDfuPrepareFlash()\r\n");
  int8_t returnValue = 0;

  switch(fwType)
  {
    case OTA_DFU_FW_TYPE_MCU:
      returnValue = OtaDfuEraseMcuSectors();

      break;

    case OTA_DFU_FW_TYPE_BLE:
      otaFwNum = OtaDfuGetOtaImgNum(PNI_FOTA_BLE_ACTIVE_IMG_NUM);
      returnValue = OtaDfuEraseBleSectors(otaFwNum);
      break;

    case OTA_DFU_FW_TYPE_A2:
      otaFwNum = OtaDfuGetOtaImgNum(PNI_FOTA_RTI_ACTIVE_IMG_NUM);
      returnValue = OtaDfuEraseRtiSectors(otaFwNum);
      break;

    case OTA_DFU_FW_TYPE_MFG_TEST:
      PNI_PRINTF("[DBG]Start Manufacture Test...\r\n");
      returnValue = OTA_DFU_OK;
      break;

    default:
      PNI_PRINTF("[ERROR]OtaDfuPrepareFlash(), incorrect firmware type(%u)\r\n", fwType);
      returnValue = OTA_DFU_ERR_INCORRECT_FW_TYPE;
  }
  return returnValue;
}

/**
 * @brief Function to generate OTA DFU result
 * @param none
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuResult(uint8_t result)
{
  PNI_PRINTF("[DBG] OtaDfuResult()\r\n");
  int8_t returnValue=0;

  /* Signal that we are ready sending back the CRC value*/
  BufferToWrite[0] = OTA_PARKING_DFU_CMD_RESULT;
  BufferToWrite[1] = 0x00;
  BufferToWrite[2] = result;
  BytesToWrite = 3;
  Dfu_Ack(BufferToWrite,BytesToWrite);

  return returnValue;
}

/**
 * @brief Function for handle DFU Stop CMD
 * @param uint8_t *att_data attribute data
 * @param int32_t data_length length of the data
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuStopCmdHandler(uint8_t * att_data, int32_t data_length)
{
  //PNI_PRINTF("[DBG] OtaDfuStopCmdHandler()\r\n");
  int8_t returnValue=0;
  totalFotaTime = Clock_Time() - totalFotaTime;

  if (curFrameId != totalFwFrames)
  {
    /* if running manufacture testing, send back fail */
    if (fwType == OTA_DFU_FW_TYPE_MFG_TEST)
      OtaDfuResult(OTA_PARKING_DFU_RESULT_FAIL);

    /*
     * some error occur caused App to stop upload in middle of FOTA,
     * Don't report any error, else it will be in endless loop
     */
    returnValue = OTA_DFU_ERR_FOTA_STOPPED;
  }
  else if (fwType == OTA_DFU_FW_TYPE_MFG_TEST)
  {
    // TODO: report manufacture testing data
    if (mfgTestResult == MFG_TEST_PASS)
    {
      PNI_PRINTF("[DBG] MFG Test -- Pass\r\n");
      OtaDfuResult(OTA_PARKING_DFU_RESULT_PASS);
    }
    else if (mfgTestResult == MFG_TEST_FAIL)
    {
      PNI_PRINTF("[DBG] MFG Test -- Fail\r\n");
      OtaDfuResult(OTA_PARKING_DFU_RESULT_FAIL);
    }
    PNI_PRINTF("[DBG] Total FOTA Time: %u:%u.%u\r\n", (totalFotaTime/1000)/60,
          (totalFotaTime/1000)%60, totalFotaTime%1000);


    returnValue = mfgTestResult;
  }
  else
  {
    PNI_PRINTF("[DBG] Total FOTA Time: %u:%u.%u\r\n", (totalFotaTime/1000)/60,
      (totalFotaTime/1000)%60, totalFotaTime%1000);

    /* we change the BLE connIntervalMin and connIntervalMax,
     * should we change to other value after firmware finish download?
     *
     * No for now, since we are going to reboot the board to run the new firmware
     */
    PNI_PRINTF("Checking FOTA CRC...");

    // since android side padded the firmware to 16bytes, find the padding for CRC calculation
    uint8_t addPadding = 0;
    if ((SizeOfUpdateBlueFW % OTA_PARKING_DFU_DATA_SIZE) != 0)
      addPadding = OTA_PARKING_DFU_DATA_SIZE - (SizeOfUpdateBlueFW % OTA_PARKING_DFU_DATA_SIZE);

    // make sure the CRC in the flash match the one we calculate from the data
    uint32_t flashCrc = 0;
    uint8_t rdBuf[OTA_PARKING_DFU_DATA_SIZE];

    // reset the CRC
    __HAL_CRC_DR_RESET(&hcrc);

    // find starting address in the SPI flash
    uint32_t startAddr = 0;
    if (fwType == OTA_DFU_FW_TYPE_MCU)
      startAddr = PNI_FOTA_EXTFLAH_IMG1_ADDR_START + ((otaFwNum - 1) * PNI_FOTA_FW_MAX_SIZE);
    else if (fwType == OTA_DFU_FW_TYPE_BLE)
      GET_FOTA_BLE_sFLASH_START_ADDR(startAddr, otaFwNum);
    else if (fwType == OTA_DFU_FW_TYPE_A2)
      GET_FOTA_RTI_sFLASH_START_ADDR(startAddr, otaFwNum);
      //startAddr = PNI_FOTA_EXTFLAH_RTI1_ADDR_START;
    else
      PNI_PRINTF("[ERROR] OtaDfuStopCmdHandler(), wrong firmware type\r\n");

    // read back value from SPI flash to check the CRC
    for (uint32_t i = 0; i < SizeOfUpdateBlueFW + addPadding; i = i + 16)
    {
      sFLASH_ReadBuffer(rdBuf, startAddr + i, OTA_PARKING_DFU_DATA_SIZE);

      flashCrc = HAL_CRC_Accumulate(&hcrc, (uint32_t *)&rdBuf[0], (OTA_PARKING_DFU_DATA_SIZE / 4));

      #if 0 // enable to verify crc and data
      PNI_PRINTF("flash crc [0x%.08X]\r\n", flashCrc);
      PNI_PRINTF("[0x%.08X:]", startAddr + i);

      //address = address + 16;
      for (int i = 0; i < 16; i++)
      {
        PNI_PRINTF("0x%.02X ", rdBuf[i]);
      }
      PNI_PRINTF("\r\n");
      #endif
    }

    #if FOTA_TEST_4
    /***** FOTA TEST CASE 4 ****/
    if (only_one_time++ == 0)
      flashCrc = 0;
    #endif

    if (flashCrc != fwDataCrc)
    {
      // TODO: handle CRC not matching?
      PNI_PRINTF("Not match(fwCrc = 0x%.08X vs flashCrc = 0x%.08X)\r\n", fwDataCrc, flashCrc);
      returnValue = OTA_DFU_ERR_CRC_NOT_MATCH;
    }
    else
    {
      PNI_PRINTF("Match(0x%.08X)\r\n", fwDataCrc);

      returnValue = OtaDfuConfigSector();

    /*********** update module firmware ****************/
      if (fwType == OTA_DFU_FW_TYPE_BLE)
      {
        /* to work with Alex's app, we need to send result before upgrade BLE FW */
        OtaDfuResult(OTA_PARKING_DFU_RESULT_PASS);

        OtaDfuUpgradeBleStack(SizeOfUpdateBlueFW);

        /* for alex app, we need to reboot the board after finish flashing BLE firmware */
        /*********** reboot ParkingXdot ****************/
        if (returnValue == OTA_DFU_OK)
        {
          /* reset the board in 5 sec */
          PNI_PRINTF("PNI Parking Module will restart in 5 seconds\r\n");

          Clock_Wait(5000);
          HAL_NVIC_SystemReset();
        }
      }

      if (fwType == OTA_DFU_FW_TYPE_A2)
      {
        if (OtaDfuUpgradeRtiFw() < 0)
          OtaDfuResult(OTA_PARKING_DFU_RESULT_FAIL);
        else
          /* to work with Alex's app, we need to send result after finish upgrade RTI FW */
          OtaDfuResult(OTA_PARKING_DFU_RESULT_PASS);

      }

      if (fwType == OTA_DFU_FW_TYPE_MCU)
      {
        /* update firmware is done in bootloader */

        /* to work with Alex's app, only send the result back */
        OtaDfuResult(OTA_PARKING_DFU_RESULT_PASS);
      }
    }
  }
  return returnValue;
}

/**
 * @brief Function for handle DFU Data CMD
 * @    DFU Data CMD format: <0x02, 0x??><frameId(2byte)><Data Frame(16 bytes)>
 * @param uint8_t *att_data attribute data
 * @param int32_t data_length length of the data
 * @retval int8_t Return value for checking purpouse (>1/-Value == frameId/Error)
 */
static int8_t OtaDfuDataCmdHandler(uint8_t * att_data, int32_t data_length, uint16_t * curFrameId)
{
  //PNI_PRINTF("[DBG] OtaDfuDataCmdHandler()\r\n");
  int8_t returnValue = 0;
  //uint16_t frameId = 0;
  uint8_t *PointerByte = (uint8_t*)curFrameId;
  PointerByte[0] = att_data[2];
  PointerByte[1] = att_data[3];

  if (*curFrameId > totalFwFrames)
  {
    PNI_PRINTF("[ERROR] too many frames, frame Id(%u) > total frames(%u)\r\n",
        *curFrameId, totalFwFrames);
    returnValue = OTA_DFU_ERR_INCORRECT_FRAME_ID;
  }
  else if (fwType == OTA_DFU_FW_TYPE_MFG_TEST)
  {
    mfgTestResult = OtaDfuCheckMfgData(*curFrameId, &att_data[OTA_PARKING_DFU_HEADER_SIZE], OTA_PARKING_DFU_DATA_SIZE);
    returnValue = mfgTestResult;
  }
  else
  {
/*    PNI_PRINTF("[DBG] frame Id(%u):", *curFrameId);
    for(int i = 0; i < OTA_PARKING_DFU_DATA_SIZE; i++)
    {
      saveTempFw[i] = att_data[OTA_PARKING_DFU_HEADER_SIZE+i];
      PNI_PRINTF("0x%.02X ", saveTempFw[i]);
    }
    PNI_PRINTF("\r\n");*/
    // TODO: for debug only
    if (*curFrameId == 1)
    {
      PNI_PRINTF("[DBG] FOTA External Flash programing(ota%u)", otaFwNum);
    }
    else if (((*curFrameId) % 10) == 0)
    {
      PNI_PRINTF(".");
    }
    else if (*curFrameId == totalFwFrames)
    {
      PNI_PRINTF("Last frame\r\n");
    }

    // find starting address in the SPI flash
    uint32_t startAddr = 0;
    if (fwType == OTA_DFU_FW_TYPE_MCU)
    {
      startAddr = PNI_FOTA_EXTFLAH_IMG1_ADDR_START + ((otaFwNum - 1) * PNI_FOTA_FW_MAX_SIZE) + \
                          (((*curFrameId) - 1) * OTA_PARKING_DFU_DATA_SIZE);
    }
    else if (fwType == OTA_DFU_FW_TYPE_BLE)
    {
      GET_FOTA_BLE_sFLASH_FRAME_ADDR(startAddr, otaFwNum, *curFrameId);
    }
    else if (fwType == OTA_DFU_FW_TYPE_A2)
    {
      GET_FOTA_RTI_sFLASH_FRAME_ADDR(startAddr, otaFwNum, *curFrameId);
      //startAddr = PNI_FOTA_EXTFLAH_RTI1_ADDR_START + (((*curFrameId) - 1) * OTA_PARKING_DFU_DATA_SIZE);
    }
    else
    {
      PNI_PRINTF("[ERROR]OtaDfuDataCmdHandler(), Wrong fwType\r\n");
    }

    OtaDfuWriteFlash(startAddr, &att_data[OTA_PARKING_DFU_HEADER_SIZE], OTA_PARKING_DFU_DATA_SIZE);

    // store CRC while saving the data to flash, so later we can do comparison
    fwDataCrc = HAL_CRC_Accumulate(&hcrc, (uint32_t *)&att_data[OTA_PARKING_DFU_HEADER_SIZE], (OTA_PARKING_DFU_DATA_SIZE / 4));
    #if 0 // verify crc and data
    PNI_PRINTF("flash crc [0x%.08X]\r\n", fwDataCrc);
    PNI_PRINTF("[0x%.08X:]", startAddr);

    //address = address + 16;
    for (int i = OTA_PARKING_DFU_HEADER_SIZE; i < (OTA_PARKING_DFU_HEADER_SIZE+16); i++)
    {
      PNI_PRINTF("0x%.02X ", att_data[i]);
    }
    PNI_PRINTF("\r\n");
    #endif

    returnValue = OTA_DFU_OK;
  }

  return returnValue;
}

/**
 * @brief Function for checking the firmware size
 * @param uint16_t fwType
 * @param int32_t fwsize
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuCheckFwSize(uint16_t fwType, uint32_t fwSize)
{
  int8_t result = 0;
  switch (fwType)
  {
    case OTA_DFU_FW_TYPE_MCU:
      if (fwSize < FOTA_DFU_MAX_PROG_SIZE)
        result = OTA_DFU_OK;
      else
        result = OTA_DFU_ERR_SIZE_OF_FW;
    break;

    case OTA_DFU_FW_TYPE_BLE:
      if (fwSize == FOTA_DFU_MAX_BLE_SIZE)
        result = OTA_DFU_OK;
      else
        result = OTA_DFU_ERR_SIZE_OF_FW;
    break;

    case OTA_DFU_FW_TYPE_A2:
          if (fwSize < FOTA_DFU_MAX_RTI_SIZE)
            result = OTA_DFU_OK;
          else
            result = OTA_DFU_ERR_SIZE_OF_FW;
    break;

    case OTA_DFU_FW_TYPE_MFG_TEST:
      if (fwSize == MFG_FW_SIZE)
        result = OTA_DFU_OK;
      else
        result = OTA_DFU_ERR_SIZE_OF_FW;
    break;

    // below still not supported, return error when loading these firmware
    case OTA_DFU_FW_TYPE_XDOT:
    default:
      result = OTA_DFU_ERR_SIZE_OF_FW;
  }

  return result;
}

/**
 * @brief Function for handle DFU Start CMD
 * @param uint8_t *att_data attribute data
 * @param int32_t data_length length of the data
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuStartCmdHandler(uint8_t * att_data, int32_t data_length)
{
  PNI_PRINTF("[DBG] OtaDfuStartCmdHandler()\r\n");
  int8_t returnValue=0;
  uint8_t *PointerByte;

  // initialize all global variables
  SizeOfUpdateBlueFW = 0;
  totalFwFrames = 0;
  fwDataCrc = 0;
  otaFwNum = 0;
  fwType = 0;

  PointerByte = (uint8_t*) &SizeOfUpdateBlueFW;
  PointerByte[0]=att_data[2];
  PointerByte[1]=att_data[3];
  PointerByte[2]=att_data[4];
  PointerByte[3]=att_data[5];

  PointerByte = (uint8_t*) &fwType;
  PointerByte[0]=att_data[10];
  PointerByte[1]=att_data[11];

  if (OtaDfuCheckFwSize(fwType, SizeOfUpdateBlueFW) < 0)
  {
    PNI_PRINTF("[ERROR] fw Type(%d) - OTA Fw size %ld\r\n", fwType, SizeOfUpdateBlueFW);
    returnValue = OTA_DFU_ERR_SIZE_OF_FW;
  }
  else
  {
    PointerByte = (uint8_t*) &totalFwFrames;
    PointerByte[0]=att_data[6];
    PointerByte[1]=att_data[7];

    uint16_t fwCrc;
    PointerByte = (uint8_t*) &fwCrc;
    PointerByte[0]=att_data[8];
    PointerByte[1]=att_data[9];

    PNI_PRINTF("[DBG]: size:%ld, frame count = %u, CRC = %.04X, fwType = %.04X\r\n",
        SizeOfUpdateBlueFW, totalFwFrames, fwCrc, fwType);

    /* Reduce the connection interval */
    returnValue = Change_BLE_Min_Max_Conn_Interval(
                  OTA_DFU_BLE_CONN_INTERVAL_MIN /* interval_min*/,
                  OTA_DFU_BLE_CONN_INTERVAL_MAX /* interval_max */);

    if (returnValue != BLE_STATUS_SUCCESS)
    {
      // TODO: do we need to do something?
      returnValue = OTA_DFU_ERR_CHG_BLE_CONN_INTVAL;
    }
    else
    {
      if ((returnValue = OtaDfuPrepareFlash(fwType)) < 0)
      {
        // error, return value will trigger ERR ACK
      }
      else
      {
        // always reset the CRC when starting new upload
        __HAL_CRC_DR_RESET(&hcrc);

        totalFotaTime = Clock_Time();
        returnValue = OTA_DFU_OK;
      }
    }
  }
  return returnValue;
}

/**
 * @brief Function to generate ACK for DFU Start CMD
 * @param none
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuStartCmdAck()
{
  //PNI_PRINTF("[DBG] OtaDfuStartCmdAck()\r\n");
  int8_t returnValue=0;

  /* Signal that we are ready sending back the CRC value*/
  BufferToWrite[0] = OTA_PARKING_DFU_CMD_ACK;
  BufferToWrite[1] = 0x00;
  BufferToWrite[2] = OTA_PARKING_DFU_ACK_OK;
  BufferToWrite[3] = 0x00;
  BufferToWrite[4] = 0x00;
  BytesToWrite = 5;
  Dfu_Ack(BufferToWrite,BytesToWrite);

  return returnValue;
}

/**
 * @brief Function to generate ACK for DFU Data CMD
 * @param uint8_t current process frameId
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuDataCmdAck(uint16_t curFrameId)
{
  //PNI_PRINTF("[DBG] OtaDfuDataCmdAck()\r\n");
  int8_t returnValue=0;

  /* Signal that we are ready sending back the CRC value*/
  BufferToWrite[0] = OTA_PARKING_DFU_CMD_ACK;
  BufferToWrite[1] = 0x00;
  BufferToWrite[2] = OTA_PARKING_DFU_ACK_OK; /* PARKING_DFU_ACK_OK */

  uint8_t *PointerByte = (uint8_t*)&curFrameId;
  BufferToWrite[3] = PointerByte[0];
  BufferToWrite[4] = PointerByte[1];
  BytesToWrite = 5;
  Dfu_Ack(BufferToWrite,BytesToWrite);

  return returnValue;
}

/**
 * @brief Function to generate ACK for DFU Data CMD
 * @param uint8_t current process frameId
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
static int8_t OtaDfuCmdErrAck()
{
  PNI_PRINTF("[DBG] OtaDfuDataCmdAck()\r\n");
  int8_t returnValue=0;

  /* Signal that we are ready sending back the CRC value*/
  BufferToWrite[0] = OTA_PARKING_DFU_CMD_ACK;
  BufferToWrite[1] = 0x00;
  BufferToWrite[2] = OTA_PARKING_DFU_ACK_ERR;
  BufferToWrite[3] = 0;
  BufferToWrite[4] = 0;
  BytesToWrite = 5;
  Dfu_Ack(BufferToWrite,BytesToWrite);

  return returnValue;
}

/**
 * @brief  This function makes the parsing of the OTA DFU Console Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t SendItBack true/false
 */
static void OtaDfuConsoleCommandParsing(uint8_t * att_data, uint8_t data_length)
{
/*  PNI_PRINTF("[DBG] DfuConsoleCommandParsing(), \r\n\tdata: ");
  for(int i = 0; i < data_length; i++)
  {
    PNI_PRINTF("0x%.02X ", att_data[i]);
  }
  PNI_PRINTF("\r\n");***********************************/
  uint8_t dfuCmd = att_data[0];
  if (OtaDfuCheckCrc8(att_data[1], att_data+2, data_length - 2) < 0)
  {
    // send ERROR ACK
    OtaDfuCmdErrAck();
  }
  else
  {
    switch (dfuCmd)
    {
      case OTA_PARKING_DFU_CMD_START:
        if (OtaDfuStartCmdHandler(att_data, data_length) < 0)
        {
          OtaDfuCmdErrAck();
        }
        else
        {
          OtaDfuStartCmdAck();
        }
        break;

      case OTA_PARKING_DFU_CMD_DATA:
        if (OtaDfuDataCmdHandler(att_data, data_length, &curFrameId) < 0)
          OtaDfuCmdErrAck();
        else
          OtaDfuDataCmdAck(curFrameId);
        break;

      case OTA_PARKING_DFU_CMD_STOP:
      {
        int8_t result;
        result = OtaDfuStopCmdHandler(att_data, data_length);
        if (result < 0)
        {
          // send result only when firmware CRC not mathing.
          if (result == OTA_DFU_ERR_CRC_NOT_MATCH)
            OtaDfuResult(OTA_PARKING_DFU_RESULT_FAIL);

          /*
           * if data integraty error, then we already sent error message, don't send another one
           * will cause endless loop ERR->STOP->ERR....
           */
        }

        /* if we finish with mfg testing notify state machine */
        if (fwType == OTA_DFU_FW_TYPE_MFG_TEST)
        {
          Parking_SetFsm(PARK_STATE_MFG_TEST_DONE);
        }
        break;
      }
      case OTA_PARKING_DFU_CMD_ACK:
        PNI_PRINTF("[ERROR] Should not receive any DFU ACK\r\n");
        break;

      case OTA_PARKING_DFU_CMD_REBOOT:
        OtaDfuResult(OTA_PARKING_DFU_RESULT_REBOOT);

        PNI_PRINTF("[DBG] Rebooting system...\r\n");
        /* reset the board in 5 sec */
        Clock_Wait(3000);
        HAL_NVIC_SystemReset();
        break;

      default:
        PNI_PRINTF("[ERROR] UNKNOWN DFU command\r\n");
        break;
    }
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 * With this function it's possible to understand if one application
 * is subscribed or not to the one service
 * @param uint16_t att_handle Handle of the attribute
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
static void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length)
{
  if (attr_handle == dfuRxPktCharHandle + 1)
  {
    // when user press "UPLOAD" in the DFU page on ParkingDfuFotaApp
    OtaDfuConsoleCommandParsing(att_data, data_length);
  }
  else if (attr_handle == dfuTxPktCharHandle + 2)
  {
    PNI_PRINTF("[DBG] dfuTxPktCharHandle + 2\r\n\tatt_data:");
    for (int i = 0; i < data_length; i++)
    {
      PNI_PRINTF("0x%.02X ", att_data[i]);
    }
    PNI_PRINTF("\r\n");

    if (att_data[0] == 01)
    {
      W2ST_ON_CONNECTION(W2ST_CONNECT_DFU_TERM);
    }
    else if (att_data[0] == 0)
    {
      W2ST_OFF_CONNECTION(W2ST_CONNECT_DFU_TERM);
    }
  }
  else
  {
    PNI_PRINTF("[ERROR] UNKOWN Handler\r\n");
  }
}

/**
 * @brief Function for update the configuration with correct MCU OTA num to use
 * @param none
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
int8_t OtaDfuUpdateMcuOtaImgNum()
{
  uint8_t currOtaNum = 0;
  uint8_t otaBoot = 0;
  uint8_t result = OTA_DFU_OK;

  sFLASH_ReadBuffer(&currOtaNum, PNI_FOTA_CUR_OTA_NUM_ADDR, 1);

  sFLASH_ReadBuffer(&otaBoot, PNI_FOTA_STATUS_ADDR, 1);

  PNI_PRINTF("[DBG] Current OTA num(%u) and OTA boot status (%u)\r\n", currOtaNum, otaBoot);

  //otaBoot = PNI_FOTA_BOOT_STATUS_BOOTLOADER;
  //lastOtaNum = 2;

  // only update ota number and status if the first time we run OTA image
  if (otaBoot == OTA_STATUS_UNSTABLE)
  {
    sFLASH_EraseSector(0x00000000);

    // this firmware is stable
    otaBoot = OTA_STATUS_STABLE;
    sFLASH_WriteBuffer(&otaBoot, PNI_FOTA_STATUS_ADDR, 1);

    // switch OTA image to other one.
    uint8_t switchOtaNum = (currOtaNum == 1)? 2 : 1;
    //PNI_PRINTF("[DBG] switch OTA image to image-%u\r\n", switchOtaNum);
    sFLASH_WriteBuffer(&switchOtaNum, PNI_FOTA_CUR_OTA_NUM_ADDR, 1);
  }

  return result;
}

#if 0
/**
 * @brief Function for Updating the MOTENV1 Firmware
 * @param uint32_t *SizeOfUpdate Remaining size of the firmware image [bytes].
 * @param uint8_t *att_data attribute data
 * @param int32_t data_length length of the data
 * @param uint8_t WriteMagicNum 1/0 for writing or not the magic number
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
int8_t UpdateFWBlueMS(uint32_t *SizeOfUpdate,uint8_t * att_data, int32_t data_length,uint8_t WriteMagicNum)
{
  int8_t returnValue=0;
  // remember to disable below message when testing FOTA, the printf will cause issue for FOTA
  // PNI_PRINTF("[DBG][FOTA]Update FW to flash, remain size = %u, length = %u\r\n", *SizeOfUpdate, data_length);

  /* Save the Packed received */
  // TODO: L073 will not be updating the OTA in the Flash, need to write into the SPI flash instead of into internal memory
  if (data_length > (*SizeOfUpdate))
  {
    PNI_PRINTF("  [ERROR][FOTA] Something wrong with upgrade, data length > size of update\r\n");
    returnValue = -1;
    *SizeOfUpdate = 0;
  }
  else
  {
    // TODO: save OTA packed to flash
    static uint32_t Address = PNI_FOTA_INTFLAH_ADDRESS_START;
    int32_t counter;
    uint32_t uwCRCValue = 0;
    uint32_t writeData = 0;
    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    #if 1 /* for internal flash testing */
    // Phase 1: test internal flash with smaller image
    for(counter = 0; counter < data_length; counter = counter + 4)
    {
      writeData = (att_data[counter+3]<<24) + (att_data[counter+2]<<16) + (att_data[counter+1]<<8)+ att_data[counter];
      //writeData = (att_data[counter]<<24) + (att_data[counter+1]<<16) + (att_data[counter+2]<<8)+ att_data[counter+3];
      //if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, Address, att_data[counter]) == HAL_OK)
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, writeData) == HAL_OK)
      {
        //PNI_PRINTF("[DBG] program %u\r\n", Address);
       Address = Address + 4;
      }
      else
      {
        PNI_PRINTF("[ERROR] flash program, at size %u\r\n", *SizeOfUpdate);

        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error
           FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
        Error_Handler();
      }
    }
    #endif

    // Reduce the remaing bytes for OTA completition
    *SizeOfUpdate -= data_length;

   //if (*SizeOfUpdate < 6500)
      //PNI_PRINTF("%u ", *SizeOfUpdate);
    //PNI_PRINTF("H ");

    if (*SizeOfUpdate == 0)
    {
      // now we have received whole firmware
      PNI_PRINTF("  [DBG][FOTA]Verifying CRC...");

      // PHASE1: verify internal flash OTA image CRC
      if (WriteMagicNum == 1)
      {
        /* Compute the CRC */
        #if 1 /* for internal flash testing */
        //printf("[DBG] compare size: %u vs %u\r\n", SizeOfUpdateBlueFW/4, SizeOfUpdateBlueFW>>2);
        uwCRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)PNI_FOTA_INTFLAH_ADDRESS_START, (SizeOfUpdateBlueFW/4)/*SizeOfUpdateBlueFW>>2*/);
        PNI_PRINTF("  [DBG]size: %u\r\n", SizeOfUpdateBlueFW);
        #endif

        if (uwCRCValue == AspecteduwCRCValue)
        {
          returnValue = 1;
          PNI_PRINTF("Correct(0x%X)\r\n", uwCRCValue);

          // TODO: write magic number, if CRC is correct
          Address = PNI_FOTA_INTFLAH_MAGIC_ADDRESS;
          if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, PNI_FOTA_MAGIC_NUM)!=HAL_OK)
          {
            /* Error occurred while writing data in Flash memory.
               User can add here some code to deal with this error
               FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
            Error_Handler();
          }
          else
          {
            //PNI_PRINTF("OTA %s will be installed at next board reset\r\n",MOTENV_PACKAGENAME);
            PNI_PRINTF("OTA will be installed at next board reset\r\n");
          }
        }
        else
        {
          PNI_PRINTF("Wrong CRC(0x%X vs 0x%X)!!\r\n", uwCRCValue, AspecteduwCRCValue);
          returnValue = -1;
        }
      }
    }

    /* Lock the Flash to disable the flash control register access (recommended
       to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();
  }

  return returnValue;
}
#endif

/**
 * @brief  Add the Dfu service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_Dfu_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];

  COPY_DFU_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3*2,&DfuServHandle);

  if (ret != BLE_STATUS_SUCCESS)
  {
    goto fail;
  }

  COPY_DFU_CTRL_POINT_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(DfuServHandle, UUID_TYPE_128, uuid, PNI_DFU_MAX_CHAR_LEN,
                         /*CHAR_PROP_NOTIFY| */CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE /*| CHAR_PROP_READ*/,
                         ATTR_PERMISSION_NONE,
                         GATT_NOTIFY_ATTRIBUTE_WRITE/* | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP*/,
                         16, CHAR_VALUE_LEN_VARIABLE, &dfuRxPktCharHandle);

  if (ret != BLE_STATUS_SUCCESS)
  {
     goto fail;
  }

  COPY_DFU_PACKET_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(DfuServHandle, UUID_TYPE_128, uuid, PNI_DFU_MAX_CHAR_LEN,
                         CHAR_PROP_NOTIFY/*| CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE | CHAR_PROP_READ*/,
                         ATTR_PERMISSION_NONE,
                         0/*GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP*/,
                         16, CHAR_VALUE_LEN_VARIABLE, &dfuTxPktCharHandle);

  if (ret != BLE_STATUS_SUCCESS)
  {
     goto fail;
  }

  #if 0
  COPY_DFU_VER_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(DfuServHandle, UUID_TYPE_128, uuid, W2ST_CONSOLE_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, CHAR_VALUE_LEN_VARIABLE, &DfuVerCharHandle);

  if (ret != BLE_STATUS_SUCCESS)
  {
     goto fail;
  }
  #endif
  return BLE_STATUS_SUCCESS;

fail:
  //PNI_PRINTF("Error while adding Console service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Puts the device in connectable mode.
 * @param  None
 * @retval None
 */
void setConnectable(PNI_RadioDeviceId *radio_id)
{
  char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,NAME_PNI_BLE};

  uint8_t manuf_data[] = {
    2,0x0A,0x00 /* 0 dBm */, // Trasmission Power
    //8,0x09,NAME_MOTENV, // Complete Name
    sizeof(local_name), AD_TYPE_COMPLETE_LOCAL_NAME,NAME_PNI_BLE,  // PNI DFU name
    15,0xff,
    //COMPANY_ID,                             // should be company id/manufacture id
    PNI_PARKING_HW_INFO,                      // for now change it to board type(2 bytes)
    PNI_PARKING_SW_INFO,                      // PNI version(4 bytes)

    // lora device id instead of BT MAC address
    //bdaddr[5], bdaddr[4], bdaddr[3], bdaddr[2], bdaddr[1], bdaddr[0]
    radio_id->bytes[0], radio_id->bytes[1], radio_id->bytes[2], radio_id->bytes[3],
    radio_id->bytes[4], radio_id->bytes[5], radio_id->bytes[6], radio_id->bytes[7]
  };

  //AD_TYPE_MANUFACTURER_SPECIFIC_DATA
  /* disable scan response */
  hci_le_set_scan_resp_data(0,NULL);
  uint8_t result;

  if ( CHECK_BLE_FLAG_SET(BLE_FLAG_PUBLIC_MAC) )
    result = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR,
                NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);
  else
    result = aci_gap_set_discoverable(ADV_IND, 0, 0, STATIC_RANDOM_ADDR,
                NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);

  if (result != ERR_CMD_SUCCESS)
  {
    PNI_PRINTF("[ERROR] problem set discoverable, error = %u\r\n", result);
  }

  /* Send Advertising data */
  result = aci_gap_update_adv_data(sizeof(manuf_data), manuf_data);
  if (result != ERR_CMD_SUCCESS)
  {
    PNI_PRINTF("[ERROR] update adv data error, error = %u\r\n", result);
  }
}

/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void *pckt Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

  //PNI_PRINTF("[DBG]HCI_Event_CB()\r\n");
  if(hci_pckt->type != HCI_EVENT_PKT) {
    return;
  }

  switch(event_pckt->evt){

  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;

      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){
      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          PNI_PRINTF("[DBG] received EVT_BLUE_GATT_READ_PERMIT_REQ event!!\r\n");
          //evt_gatt_read_permit_req *pr = (void*)blue_evt->data;
          //Read_Request_CB(pr->attr_handle);
        }
        break;
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
         evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
         Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
        }
        break;
      }
    }
    break;
  }
}

