/**
* @file         mpc9808.c
*
* @brief        PNI mpc9808 functions
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

#include "mpc9808.h"

uint32_t Mpc9808_i2c_read_value(uint8_t registerAddress, uint8_t* value)
{
  uint16_t status = 0;
  uint8_t i2cAddr = MPC9808_ADDRESS;

  // TODO: I2C DMA mode, cause FIFO data problem, move back to none DMA mode for now
  status = HAL_I2C_Mem_Read(&MPC9808_I2C_HANDLER,i2cAddr, registerAddress, I2C_MEMADD_SIZE_8BIT, value, 1, 1000);
  //status = HAL_I2C_Mem_Read_DMA(&hi2c1,I2cAddr, registerAddress, I2C_MEMADD_SIZE_8BIT, value, 1);//, 1000);

  return status;
}

uint32_t Mpc9808_i2c_read(uint8_t registerAddress, uint8_t* buffer, uint16_t length)
{
  uint16_t status = 0;
  uint8_t i2cAddr = MPC9808_ADDRESS;

  // TODO: I2C DMA mode, cause FIFO data problem, move back to none DMA mode for now
  status = HAL_I2C_Mem_Read(&MPC9808_I2C_HANDLER,i2cAddr, registerAddress, I2C_MEMADD_SIZE_8BIT, buffer, length, 1000);
  //status = HAL_I2C_Mem_Read_DMA(&hi2c1,I2cAddr, registerAddress, I2C_MEMADD_SIZE_8BIT, buffer, length);//, 1000);

  return status;
}

uint32_t Mpc9808_i2c_write_value(uint8_t registerAddress, uint8_t value)
{
  uint16_t status = 0;
  uint8_t i2cAddr = MPC9808_ADDRESS;

  // TODO: I2C DMA mode, cause FIFO data problem, move back to none DMA mode for now
  //if (I2C_WriteMultiReg(registerAddress, &value, 1, A2I2cAddr) == HAL_OK)
  status = HAL_I2C_Mem_Write(&MPC9808_I2C_HANDLER, i2cAddr, registerAddress, 1, &value, 1, 1000);
  //status = HAL_I2C_Mem_Write_DMA(&hi2c1, A2I2cAddr, registerAddress, 1, &value, 1);//, 1000);

  return status;
}

uint8_t Mpc9808_ready()
{
  uint16_t i2cAddr = MPC9808_ADDRESS;
  // check if Sentral is Ready for communication
  HAL_StatusTypeDef x = HAL_I2C_IsDeviceReady(&MPC9808_I2C_HANDLER,(uint16_t)i2cAddr,1,1000);

  while(x != HAL_OK)
  {
    if(x == HAL_ERROR)
    {
      PNI_PRINTF("   I2C Device(%d) unable to communicate\r\n", i2cAddr);
      Error_Handler();
      return 0;
    }
    if(x == HAL_BUSY)
    {
      //Clock_Wait(200);
      PNI_PRINTF("   I2C device(%d) still busy - Try again\r\n", i2cAddr);
    }
    x=HAL_I2C_IsDeviceReady(&MPC9808_I2C_HANDLER,(uint16_t)(i2cAddr>>1),1,1000);
  }

  //PNI_PRINTF("   I2C started...communicated with device(%d)\r\n", i2cAddr);
  return 1;
}

uint8_t Mpc9808_GetIdRev(uint8_t* id, uint8_t* revision)
{
  uint8_t mpcReg = 0;
  uint8_t value[2];
  uint8_t ret = 0;

  mpcReg = MPC9808_REG_DEVICE_ID;
  if ((ret = Mpc9808_i2c_read(mpcReg, value, 2)) != 0)
  {
    PNI_PRINTF("[ERROR-%u] reading Device Id\r\n", ret);
    ret = MPC9808_ERROR;
  }
  else
  {
    *id = value[0];
    *revision = value[1];
  }
  return ret;
}

/**
  * @brief  Getting Temperature value from MPC9808
  * @param  int8_t temperature value in celsius
  * @note
  * @retval uint8_t Return value (0/1 == Ok/Error)
  */
uint8_t Mpc9808_GetTemperature(int8_t* temperature)
{
  uint8_t mpcReg = 0;
  uint8_t value[2];
  uint8_t ret = 0;

  mpcReg = MPC9808_REG_TEMPERATURE;
  if ((ret = Mpc9808_i2c_read(mpcReg, value, 2)) != 0)
  {
    PNI_PRINTF("[ERROR-%u] reading Device Id\r\n", ret);
    ret = MPC9808_ERROR;
  }
  else
  {
    uint8_t upperByte = value[0];
    uint8_t lowerByte = value[1];
#if 0
    if ((upperByte & 0x80) == 0x80)
    {
      // Ta 3 TCRIT
    }

    if ((upperByte & 0x40) == 0x40)
    {
      // Ta > Tupper
    }

    if ((upperByte & 0x40) == 0x40)
    {
      // Ta < Tlower
    }
#endif
    upperByte = upperByte & 0x1F;
    if ((upperByte & 0x10) == 0x10)
    {
      // TA < 0c
      *temperature = 256 - ((upperByte * 16) + (lowerByte / 16));
    }
    else
    {
      *temperature = (upperByte * 16) + (lowerByte / 16);
    }

    //PNI_PRINTF("[DBG] 0x%.02X%.02X --> temperature = %u celsius\r\n", value[0], value[1], temperature);
  }
  return ret;
}

