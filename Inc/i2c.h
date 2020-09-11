#ifndef _I2C_H_
#define _I2C_H_
#include "stm32f10x.h"

typedef enum
{ 
  I2C_STATE_FREE,
  I2C_FIRST_EV5,
  I2C_STATE_START,
  I2C_FIRST_EV6,
  I2C_STATE_RESTART,
  I2C_SECOND_EV5,
  I2C_STATE_DATAWAIT,
  I2C_STATE_FREEWAIT
} I2C_STATES;

typedef enum
{
  SMBUS_FREE,
  SMBUS_READ_WORD,
  SMBUS_WRITE_WORD
} SMBUS_STATES;

#define WRITE 0
#define READ  1

uint8_t I2C_ReadWord(uint8_t address, uint8_t command, void (*DoneCallback)(uint16_t *pData));
#endif  //_I2C_H_

/*
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
*/