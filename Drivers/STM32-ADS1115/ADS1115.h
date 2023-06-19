#include "i2c.h"

#define ADS1115_ADDRESS             0x48
#define ADS1115_OS_START_SINGLE     0x8000
#define GAIN                        0x0A00 //Maximum
#define ADS1115_MODE_CONTINUE       0x0000
#define ADS1115_MODE_SINGLE         0x0100
#define DATA_RATE                   4      //128
#define COMP_QUE_CONVERT            0x0003
#define REG_CONVERT                 0x00
#define REG_CONFIG                  0x01
#define REG_LO_THRESH               0x02
#define REG_HI_THRESH               0x04
#define I2C_TIMEOUT                 10
#define HI_THRESH                   0x8000
#define LO_THRESH                   0x0000

void InitACD();
void RequestADC();
uint16_t GetValue();
void WriteRegister(uint8_t registerAddr, uint16_t data);

