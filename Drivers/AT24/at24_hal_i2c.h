/*
 * at24_hal_i2c.h
 *
 *  Created on: Sep,11,2015
 *      Author: Sina Darvishi
 */

#ifndef DRIVERS_MYLIB_AT24_HAL_I2C_H_
#define DRIVERS_MYLIB_AT24_HAL_I2C_H_

#include "stm32f4xx_hal.h"

int at24_HAL_WriteBytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData,uint16_t TxBufferSize);
int at24_HAL_ReadBytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t RxBufferSize);

int at24_HAL_SequentialRead(I2C_HandleTypeDef *hi2c ,uint8_t DevAddress,uint16_t MemAddress,uint8_t *pData,uint16_t RxBufferSize);

#endif /* DRIVERS_MYLIB_AT24_HAL_I2C_H_ */
