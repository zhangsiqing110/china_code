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
#ifndef PROG_EEPROM_H
#define PROG_EEPROM_H

#include <stdint.h>
#include <stdio.h>

#include "stm32l0xx_hal.h"
#include "spi_flash.h"
#include "pni_fota.h"

HAL_StatusTypeDef eeprom_i2c_write(uint8_t registerAddress, uint8_t* buffer, uint16_t length);
HAL_StatusTypeDef eeprom_i2c_read(uint16_t registerAddress, uint8_t* buffer, uint16_t length);
int32_t eeprom_erase(void);
int32_t eeprom_upload_array(void);
int32_t eeprom_same_firmware(void);
int32_t ProgramEEPROM(void);

#endif /* PROG_EEPROM_H */

