/**
* @file			ProgramEEPROM.c
*
* @brief		functions to program EEPROM
*
* @authors		Kevin Chen, Richard Bae  
* @date			02/10/2017
* @copyright	(C) 2017 PNI Corp
*
* @copyright	Disclosure to third parties or reproduction in any form
*				whatsoever, without prior written consent, is strictly forbidden
*
*/
#include <stdint.h>

#include "SENtralA2.h"
#include "pni_config.h"

#include "SENtralA2_types.h"

#include "ProgramEEPROM.h"

// enable below for debugging
#define DBG_PROG_EEPROM   0

extern I2C_HandleTypeDef hi2c1;
uint32_t RTI_FLASH_ADDRESS_START;

HAL_StatusTypeDef eeprom_transfer(uint8_t deviceAddress, uint16_t memoryAddress,
    uint8_t *buffer, uint16_t length, uint8_t read)
{
  uint16_t pageSize = 64; // TODO:make into define
  uint16_t index = memoryAddress;
  uint16_t bytesRemaining = length;
  uint16_t bytesTransferred = 0;
  uint16_t bytesToTransfer = 0;
  uint16_t page = 0;
  HAL_StatusTypeDef ret = HAL_OK;

  do {
    page = index / pageSize;
    bytesToTransfer = (page + 1) * pageSize - index;
    bytesToTransfer = (bytesToTransfer > bytesRemaining)
        ? bytesRemaining
        : bytesToTransfer;

    bytesToTransfer = MIN(bytesToTransfer, 64);

    index = (index >> 8) | (index << 8);
    if (read) {
      ret = HAL_I2C_Mem_Read(&hi2c1, deviceAddress, index,
          I2C_MEMADD_SIZE_16BIT, &buffer[bytesTransferred], bytesToTransfer,
          1000);
    } else {
      ret = HAL_I2C_Mem_Write(&hi2c1, deviceAddress, index,
          I2C_MEMADD_SIZE_16BIT, &buffer[bytesTransferred], bytesToTransfer,
          1000);
      HAL_Delay(5);  // TODO: make into define
    }

    if (ret != HAL_OK) {
      return ret;
    }

    bytesTransferred += bytesToTransfer;
    bytesRemaining -= bytesToTransfer;
    index += bytesToTransfer;
  } while (bytesRemaining > 0);

  return HAL_OK;
}

HAL_StatusTypeDef eeprom_i2c_write(uint8_t registerAddress, uint8_t* buffer, uint16_t length)
{
  HAL_StatusTypeDef status;
  if (length >1)
  {
    status = HAL_I2C_Mem_Write(&hi2c1, 0xA0, registerAddress, 1, buffer, length, 1000);
  }
  else if (length == 1){
    status = SentralWrite(registerAddress, buffer[0]);
  }

  return status;
}

HAL_StatusTypeDef eeprom_i2c_read(uint16_t registerAddress, uint8_t* buffer, uint16_t length)
{
  return eeprom_transfer(0xA0, registerAddress, buffer, length, 1);
#if 0
  HAL_StatusTypeDef status;
  status = eeprom_transfer(0xA0, registerAddress, buffer, length, 1);
  return status;
#endif
}

#define EEPROM_BUF_LEN 64         /**< number of bytes to write to EEPROM at a time; allocated on the stack */
#define CONFIG_FILE_BUF_LEN 32    /**< number of bytes to read from config file at a time while uploading to RAM; allocated on the stack; must be 4 */

int32_t eeprom_erase()
{
	uint8_t buf[sizeof(FirmwareHeader) + 2];        // add two more bytes for the EEPROM address
	uint16_t eeprom_addr;

	// reset Sentral
  PNI_PRINTF("reset Sentral...\r\n");
	uint8_t one = 0x01;
	SENtralA2_i2c_write(RESET_REQ_REG, &one, 1);

	// go into passthrough mode
	PNI_PRINTF("enter passthrough mode...\r\n");
	SENtralA2_i2c_write(PASS_THROUGH_CFG_REG, &one, 1);
  HAL_Delay(10);

	// write the header to address 0
	PNI_PRINTF("erase EEPROM header...\r\n");
	memset(buf, 0, sizeof(buf)); 
	eeprom_addr = 0;
	buf[0] = (eeprom_addr >> 8) & 0x0ff;
	buf[1] = eeprom_addr & 0x0ff;

	uint16_t len = sizeof(FirmwareHeader);
	if (eeprom_i2c_write(buf[0], &buf[1], len + 1) != HAL_OK)
	{
		PNI_PRINTF("Write eeprom error\r\n");
		return 0;
	}

	HAL_Delay(100);

	// leave passthrough mode
	uint8_t zero = 0;
	SENtralA2_i2c_write(PASS_THROUGH_CFG_REG, &zero, 1);
	HAL_Delay(10);

	// reset
	PNI_PRINTF("finished Erasing...\r\nresetting EM718x...\r\n");
	SENtralA2_i2c_write(RESET_REQ_REG, &one, 1);

	return 1;
}

int32_t eeprom_same_firmware(void)
{
  FirmwareHeader fwHeader;
  FirmwareHeader existingFwHeader;
  uint8_t cur_flash_fw = 0;

	//Get the firmware from proper flash location
	sFLASH_ReadBuffer(&cur_flash_fw, PNI_FOTA_RTI_ACTIVE_IMG_NUM, 1);
  PNI_PRINTF("Active Flash Image: %u\r\n", cur_flash_fw);
	if (cur_flash_fw == 1)
	{
		RTI_FLASH_ADDRESS_START = PNI_FOTA_EXTFLAH_RTI1_ADDR_START;
	}
	else
	{
		RTI_FLASH_ADDRESS_START = PNI_FOTA_EXTFLAH_RTI2_ADDR_START;
	}
	PNI_PRINTF("Current Flash start addr: %.8X\r\n", RTI_FLASH_ADDRESS_START);

  sFLASH_ReadBuffer((uint8_t*)&fwHeader, RTI_FLASH_ADDRESS_START, sizeof(FirmwareHeader));

  //PNI_PRINTF("enter passthrough mode...\r\n");
  uint8_t one = 1;
  SENtralA2_i2c_write(PASS_THROUGH_CFG_REG, &one, 1);
  HAL_Delay(10);

  // read EEPROM header
  eeprom_i2c_read(0, (uint8_t *) &existingFwHeader, FIRMWARE_HEADER_SIZE);
  PNI_PRINTF("Incoming EEPROM Header:\r\n");
  PNI_PRINTF("   Magic - %.4X\r\n", existingFwHeader.imageSignatureLsb + (existingFwHeader.imageSignatureMsb << 8));
  PNI_PRINTF("   CRC - %.8X\r\n", existingFwHeader.crc);
  PNI_PRINTF("   Size - %d\r\n", existingFwHeader.imageLength);
  PNI_PRINTF("Existing EEPROM Header:\r\n");
  PNI_PRINTF("   Magic - %.4X\r\n", fwHeader.imageSignatureLsb + (fwHeader.imageSignatureMsb << 8));
  PNI_PRINTF("   CRC - %.8X\r\n", fwHeader.crc);
  PNI_PRINTF("   Size - %d\r\n", fwHeader.imageLength);



  // leave passthrough mode
  uint8_t zero = 0;
  SENtralA2_i2c_write(PASS_THROUGH_CFG_REG, &zero, 1);
  HAL_Delay(10);

  // compare CRC to tell if firmware is the same
  if (existingFwHeader.crc == fwHeader.crc)
  {
    PNI_PRINTF("  EEPROM has the same firmware\r\n");
    return 1;
  }
  else
  {
    PNI_PRINTF("  EEPROM has different firmware\r\n");
    return 0;
  }
}

int32_t eeprom_upload_array(void)
{
	FirmwareHeader header;
	uint8_t buf[EEPROM_BUF_LEN + 2];                                       // add two more bytes for the EEPROM address
	uint16_t eeprom_addr;
	uint8_t *src;
	uint8_t *dst;
	uint16_t eelen;
	uint16_t len;
	uint16_t wlen;
	uint32_t wbuf[CONFIG_FILE_BUF_LEN / 4];
    FirmwareHeader FirmwareHeader;

	///////////////////////////////////////////////////////////////////////////
	// Load firmware from array
	///////////////////////////////////////////////////////////////////////////

	PNI_PRINTF("read eeprom header...\r\n");
    //Get header from SPI Flash
  //void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
    sFLASH_ReadBuffer((uint8_t*)&header, RTI_FLASH_ADDRESS_START, sizeof(FirmwareHeader));
	//check header
	PNI_PRINTF("header: EEPROMNoExec: %u, I2CClockSpeed: %u, ROMVerExp: %u, ImageLength: %u\r\n",
		header.flags.bits.EEPROMNoExec, header.flags.bits.I2CClockSpeed, header.flags.bits.ROMVerExp, header.imageLength);

	///////////////////////////////////////////////////////////////////////////
	// Upload firmware to eeprom
	///////////////////////////////////////////////////////////////////////////

	// go into passthrough mode
    
	PNI_PRINTF("enter passthrough mode...\r\n");
	uint8_t one = 1;
	SENtralA2_i2c_write(PASS_THROUGH_CFG_REG, &one, 1);
	HAL_Delay(10);

	// write the header to address 0
	PNI_PRINTF("write EEPROM header...\r\n");
	len = sizeof(FirmwareHeader);
	eeprom_addr = 0;
	src = (uint8_t *)&header;
	while (len)
	{
		buf[0] = (eeprom_addr >> 8) & 0x0ff;
		buf[1] = eeprom_addr & 0x0ff;
		if (len > EEPROM_BUF_LEN)                                      // limit to EEPROM page size
			eelen = EEPROM_BUF_LEN;
		else
			eelen = len;
		dst = &buf[2];
		for (int i = 0; i < eelen; i++)
			*(dst++) = *(src++);
		if (eeprom_i2c_write(buf[0], &buf[1], eelen + 1) != HAL_OK)
		{
			PNI_PRINTF("Write eeprom error\r\n");
			return 0;
		}

		HAL_Delay(100);
		eeprom_addr += eelen;
		len -= eelen;
	}

	// after the header, upload all the text section data from the file
    PNI_PRINTF("write text section and rest of the file...\r\n");
	int32_t upload_len = (int32_t)((uint32_t)header.imageLength);
	uint32_t upload_ofs = sizeof(FirmwareHeader);

	// upload the rest of the file
	int arrlen = upload_len;

	uint16_t fwindex = FIRMWARE_HEADER_SIZE;

  // writing

	while (arrlen > 0)
	{
        // read a section from SPI Flash and write it to EEPROM
		if (arrlen > CONFIG_FILE_BUF_LEN / 4)
			wlen = CONFIG_FILE_BUF_LEN / 4;
		else
			wlen = (uint16_t)arrlen;

        // Read fw image from Flash
        sFLASH_ReadBuffer((uint8_t*)&wbuf, RTI_FLASH_ADDRESS_START + fwindex, wlen);
       
		//Loading eeprom
    #if DBG_PROG_EEPROM
    PNI_PRINTF("load data ofs %u...\r", upload_ofs);
    #endif
		uint16_t my_wlen = wlen / 4;
		while ((upload_len > 0) && my_wlen)
		{
			buf[0] = ((upload_ofs) >> 8) & 0x0ff;
			buf[1] = (upload_ofs)& 0x0ff;
			if (my_wlen * 4 > EEPROM_BUF_LEN)                   // limit to EEPROM page size
				eelen = EEPROM_BUF_LEN;
			else
				eelen = my_wlen * 4;
			memcpy(&buf[2], wbuf, eelen);
			if (eeprom_i2c_write(buf[0], &buf[1], eelen + 1) != HAL_OK)
			{
				PNI_PRINTF("I2C bus error\r\n");
				return 0;
			}

			// wait for page to finish
			HAL_Delay(10);
			upload_ofs += eelen;
			upload_len -= (int32_t)((uint32_t)eelen);
			my_wlen -= eelen / 4;
		}

		if (my_wlen)                                                         // they gave us extra
		{
			PNI_PRINTF("invalid file length\r\n");
			return 0;
		}

		//Next block
		arrlen -= wlen;
		fwindex += wlen;
	}
  //HAL_GPIO_WritePin(LED3_PB7.PORT,LED3_PB7.PIN, GPIO_PIN_SET);
    PNI_PRINTF("done uploading firmware...\r\n");
    /*
    PNI_PRINTF("Checking contents...\r\n");
    uint8_t temp_buf;
    for (int i = 0; i < fwindex + 16; i++)
    {
        eeprom_i2c_read(i, &temp_buf, 1);
        PNI_PRINTF("[%.2X] ", temp_buf);
    }
    */
	// leave passthrough mode
	uint8_t zero = 0;
	SENtralA2_i2c_write(PASS_THROUGH_CFG_REG, &zero, 1);
	HAL_Delay(10);

	// reset
	PNI_PRINTF("finished uploading...\r\nresetting EM7181...\r\n");
	SENtralA2_i2c_write(RESET_REQ_REG, &one, 1);

  Clock_Wait(1000);

	// Read and verify Chip Status
	PNI_PRINTF("Check eeprom uploading status\r\n");
	uint8_t bootupStatus;
	SENtralA2_i2c_read(CHIP_STATUS_REG, (uint8_t*)&bootupStatus, 1);
	PNI_PRINTF("chipstatusreg: 0x%x\r\n", bootupStatus);

	if (bootupStatus & EEUPLOADDONE)
		PNI_PRINTF("eeprom upload done\r\n");

	if (bootupStatus & EEUPLOADERR)
	{
    PNI_PRINTF("[ERROR] EEPROM update error!! erase EEPROM FW header\r\n");

    eeprom_erase();

		return 0;
	}

	return 1;
}


int32_t ProgramEEPROM(void){

    // Check the device id
	displayDeviceIdRegisters();

    PNI_PRINTF("Enter EEPROM Programming\r\n");

    PNI_PRINTF("erasing EEPROM...");
    if (!eeprom_erase())
    {
      PNI_PRINTF("failed!\r\n");
    }
    else
      PNI_PRINTF("erase complete\r\n");

    PNI_PRINTF("Programming EEPROM... \r\n");
    if (!eeprom_upload_array())
    {
      PNI_PRINTF("Error uploading firmware:\r\n");
      Clock_Wait(2000);
      return 0;
    }
    else
      PNI_PRINTF("complete\r\n");
  //}
  return 1;
}

