/* | -------------------------------- R2T Team Libraries -------------------------
 * | @Created On Sep,11,2015
 * | @File Name : at24_hal_i2c
 * | @Brief : STM32 HAL Driver for AT24 eeprom series
 * | @Copyright :
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * | @Author :  Sina Darvishi
 * | @Website : R2T.IR
 * |
**/
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/**
  * @brief               : This function handles Writing Array of Bytes on the specific Address .
  * 					   This program have this feature that don't force you to use absolute 16 bytes
  * 					   you can use more than 16 bytes buffer.
  * @param  hi2c         : Pointer to a I2C_HandleTypeDef structure that contains
  *                        the configuration information for the specified I2C.
  * @param	DevAddress   : specifies the slave address to be programmed(EEPROM ADDRESS).
  * @param	MemAddress   : Internal memory address (WHERE YOU WANNA WRITE TO)
  * @param	pData	     : Pointer to data buffer
  * @param  TxBufferSize : Amount of data you wanna Write
  * @retval
  */
int at24_HAL_WriteBytes(I2C_HandleTypeDef *hi2c,uint16_t DevAddress,uint16_t MemAddress, uint8_t *pData,uint16_t TxBufferSize)
{
	/*
	 * program just get the DevAddress of the Slave (not master) and for the next step
	 * You know that the most of the EEprom address start with 0xA0
	 * give MemAddress for the location you want to write to
	 * give Data buffer so it can write Data on this location
	 */
	//Note that this function works properly to 31 bytes
	if(MemAddress+TxBufferSize > 16)
	{
		//Write to 16bytes
		while(HAL_I2C_Master_Transmit(hi2c,(uint16_t)DevAddress, pData, (uint16_t)16-MemAddress,1000)!= HAL_OK);
		//write remaining bytes
		*pData = *pData + (16-MemAddress);
		while(HAL_I2C_Master_Transmit(hi2c,(uint16_t)DevAddress, pData, (uint16_t)16-MemAddress,1000)!= HAL_OK);

	}
	else
	{
			while( (TxBufferSize-16)>0 )
			{
				//if your data is more than 16 bytes,you are here
                 while(HAL_I2C_Master_Transmit(hi2c,(uint16_t)DevAddress, pData, (uint16_t)16-MemAddress,1000)!= HAL_OK);
				 TxBufferSize-=16;
				 pData+=16;
				 MemAddress+=16;
			}
			//remaining data
            while(HAL_I2C_Master_Transmit(hi2c,(uint16_t)DevAddress, pData, (uint16_t)16-MemAddress,1000)!= HAL_OK);
	}
	return 1;
}


int at24_HAL_ReadBytes(I2C_HandleTypeDef *hi2c,uint16_t DevAddress,uint16_t MemAddress, uint8_t *pData,uint16_t RxBufferSize)
{
	int TimeOut;
	/*
	 * program just get the DevAddress of the Slave (not master) and for the next step
	 * You know that the most of the EEprom address start with 0xA0
	 * get the MemAddress for the location you want to write data on it
	 * get the Data buffer so it can write Data on this location
	 */
	//Note that this function works properly to 31bytes

    while( (RxBufferSize-16)>0 )
    {
        //if your data is more than 16 bytes,you are here
        TimeOut = 0;
         while(HAL_I2C_Mem_Read(hi2c,(uint16_t)DevAddress,(uint16_t)MemAddress,I2C_MEMADD_SIZE_8BIT,pData,(uint16_t)16,1000)!= HAL_OK && TimeOut < 10)
         {
                TimeOut++;
         }

         RxBufferSize-=16;
         pData+=16;
         MemAddress+=16;
    }
    //			//remaining data
    TimeOut = 0;
    while(HAL_I2C_Mem_Read(hi2c,(uint16_t)DevAddress,(uint16_t)MemAddress,I2C_MEMADD_SIZE_8BIT,pData,(uint16_t)RxBufferSize,1000)!= HAL_OK && TimeOut < 10)
    {
        TimeOut++;
    }

	return 1;
}

 /*
  * @brief               : This function handles Reading Array of Bytes from the specific Address .
  * 					   This program have this feature that don't force you to use absolute 16 bytes
  * 					   you can use more than 16 bytes buffer.
  * @param  hi2c         : Pointer to a I2C_HandleTypeDef structure that contains
  *                        the configuration information for the specified I2C.
  * @param	DevAddress   : specifies the slave address to be programmed(EEPROM ADDRESS).
  * @param	MemAddress   : Internal memory address (WHERE YOU WANNA READ FROM)
  * @param	pData	     : Pointer to data buffer
  * @param  TxBufferSize : Amount of data to be Read
  * @retval
  */
int at24_HAL_SequentialRead(I2C_HandleTypeDef *hi2c ,uint8_t DevAddress,uint16_t MemAddress,uint8_t *pData,uint16_t RxBufferSize)
{
	/*
	 * just like WriteByte but get what it want
	 * but maybe you should know that the Data is location you want to save data
	 * for future use
	 */
		while( (RxBufferSize-16)>0)
		{
			while(HAL_I2C_Mem_Read(&hi2c, (uint16_t)DevAddress,(uint16_t)MemAddress,I2C_MEMADD_SIZE_8BIT,pData, (uint16_t)16, (uint32_t)1000)!= HAL_OK);
			RxBufferSize-=16;
			pData+=16;
			MemAddress+=16;
		}
		while(HAL_I2C_Mem_Read(&hi2c, (uint16_t)DevAddress,(uint16_t)MemAddress,I2C_MEMADD_SIZE_8BIT,pData, (uint16_t)RxBufferSize, (uint32_t)1000)!= HAL_OK);
        /*
         * if DataRecive  is 0xFF or 255 ,this means that block was empty
         */
	   return 1;
}

 /*
  * @brief               : This function handles Erase Full chip.
  * @param  hi2c         : Pointer to a I2C_HandleTypeDef structure that contains
  *                        the configuration information for the specified I2C.
  * @retval
  */




