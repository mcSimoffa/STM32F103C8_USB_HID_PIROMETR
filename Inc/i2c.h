#ifndef _I2C_H_
#define _I2C_H_
#include "stm32f10x.h"

typedef enum
{
  I2C_STATE_START,
  I2C_STATE_ADDRESS_W,
  I2C_STATE_COMMAND,
  I2C_STATE_RESTART,
  I2C_STATE_ADDRESS_R,
  I2C_STATE_READ_LOW,
  I2C_STATE_STOP,
  I2C_STATE_TIMEOUT,
  I2C_STATE_BUSU
} I2C_STATES;
    
#define WRITE 0
#define READ  1

uint8_t I2C_ReadWord(uint8_t address, uint8_t command, void (*DoneCallback)(void));
#endif  //_I2C_H_