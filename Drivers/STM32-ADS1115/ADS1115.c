#include "ADS1115.h"
#include "i2c.h"

uint16_t ADS1115_value;

void InitADC()
{
    WriteRegister(REG_HI_THRESH, HI_THRESH);
    WriteRegister(REG_LO_THRESH, LO_THRESH);
}

void RequestADC()
{
  uint16_t config = ADS1115_OS_START_SINGLE;  // bit 15     force wake up if needed
  config |= 0;                                // bit 12-14
  config |= GAIN;                             // bit 9-11
  config |= ADS1115_MODE_SINGLE;              // bit 8
//  config |= DATA_RATE;                      // bit 5-7
  config |= COMP_QUE_CONVERT;                 // bit 0..1   ALERT mode
  WriteRegister(REG_CONFIG, config);
}

HAL_StatusTypeDef GetADCValue()
{
    uint8_t buffer[2];
    HAL_StatusTypeDef hal_result = HAL_I2C_Master_Receive(&hi2c1, (ADS1115_ADDRESS << 1), buffer, 2,  I2C_TIMEOUT);
    if (hal_result == HAL_OK)
    {
        uint16_t result = buffer[1];
        ADS1115_value = result << 8 | buffer[0]; //? порядок байт    
    }
    return hal_result;
}

void WriteRegister(uint8_t registerAddr, uint16_t data)
{
    uint8_t buffer[3];
    buffer[0] = registerAddr;
    buffer[1] = data >> 8;
    buffer[2] = data & 0xFF;
    HAL_I2C_Master_Transmit(&hi2c1, (ADS1115_ADDRESS << 1), buffer, 3,  I2C_TIMEOUT);
}

