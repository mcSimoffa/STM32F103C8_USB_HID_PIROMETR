#include "i2c.h"
#include "SWOfunction.h"
#include "additional_func.h"

I2C_STATES    i2cState = I2C_STATE_FREE;
SMBUS_STATES  smbusState = SMBUS_FREE; 
uint8_t addr;
uint8_t comm;
uint16_t data;
void (*Callback)(uint16_t *pData);

#ifdef SWO_SMBUS_LOG
  char debugSmbusBuf[256];
  char *pheadBuf;
#endif
  
/* *************************************************************
I2C1 interrupt Handler
************************************************************* */  
void I2C1_EV_IRQHandler()
{
  uint16_t sr1 = I2C1->SR1;
#ifdef SWO_SMBUS_LOG
  pheadBuf = stradd (debugSmbusBuf, "\r\n");  //new debugging string
  pheadBuf = stradd (pheadBuf, "\r\nSR1=");
  pheadBuf = itoa(sr1, pheadBuf,2,0);
#endif
  if (smbusState == SMBUS_READ_WORD)
  {
    switch(i2cState)
    {
    case I2C_FIRST_EV5:
      if(sr1 & I2C_SR1_SB)
      {
        i2cState = I2C_STATE_START;
        I2C1->DR = (addr<<1) | WRITE;
      #ifdef SWO_SMBUS_LOG
        pheadBuf = stradd (pheadBuf, "\r\nADDR | W sent");
      #endif      
      }
      break;
      
      case I2C_STATE_START:
        if (sr1 & I2C_SR1_ADDR)
        {
          i2cState = I2C_FIRST_EV6;
          (void) I2C1->SR2;
          I2C1->DR = comm;
        #ifdef SWO_SMBUS_LOG
          pheadBuf = stradd (pheadBuf, "\r\nCOMMAND sent");
        #endif
        }
        break;
       
      case I2C_FIRST_EV6:  
        if (sr1 & I2C_SR1_TXE)
        {
          i2cState = I2C_STATE_RESTART;
          I2C1->CR1 |= I2C_CR1_START;
        #ifdef SWO_SMBUS_LOG
          pheadBuf = stradd (pheadBuf, "\r\nRESTART sent");
        #endif
        }
        break;
        
      case I2C_STATE_RESTART:
        if(sr1 & I2C_SR1_SB)
        {
          i2cState = I2C_SECOND_EV5;
          I2C1->DR = (addr<<1) | READ;
        #ifdef SWO_SMBUS_LOG
          pheadBuf = stradd (pheadBuf, "\r\nADDR | R sent");
        #endif
        }
        break;
      
      case I2C_SECOND_EV5: 
        if (sr1 & I2C_SR1_ADDR)
        { 
          (void) I2C1->SR2;
          i2cState = I2C_STATE_DATAWAIT;
          I2C1->CR1 &= ~I2C_CR1_ACK;
        #ifdef SWO_SMBUS_LOG
          pheadBuf = stradd (pheadBuf, "\r\nWait wData");
        #endif
        }
        break;
       
      case I2C_STATE_DATAWAIT:
        if (sr1 & I2C_SR1_BTF)
        {
          i2cState = I2C_STATE_FREEWAIT;
          I2C1->CR1 |= I2C_CR1_STOP;
          data =  (uint8_t)I2C1->DR;
          data |= ((uint8_t)I2C1->DR) << 8;
          if (Callback)
            Callback(&data);
        #ifdef SWO_SMBUS_LOG
          pheadBuf = stradd (pheadBuf, "\r\nStop sent, Road wData");
        #endif  
        }
        break;
       
      case I2C_STATE_FREEWAIT:
        if (!(I2C1->CR1 & I2C_CR1_STOP))
        {
          i2cState = I2C_STATE_FREE;
          smbusState = SMBUS_FREE;
        #ifdef SWO_SMBUS_LOG
          pheadBuf = stradd (pheadBuf, "\r\nLine free");
        #endif  
        }
        break; 
      
    default:
      i2cState = I2C_STATE_FREE;
    } //switch(i2cState)
  }
  else if (smbusState == SMBUS_WRITE_WORD)
  {
    asm("nop");
    
    
    
  }
#ifdef SWO_SMBUS_LOG  
  putlog(debugSmbusBuf, pheadBuf);
#endif  
}// I2C1 Event

/* *************************************************************
I2C1 Errror interrupt Handler
************************************************************* */
void I2C1_ER_IRQHandler()
{
#ifdef SWO_SMBUS_LOG
  pheadBuf = stradd (debugSmbusBuf, "\r\nERROR");  //new debugging string
  pheadBuf = stradd (pheadBuf, "\r\nSR1=");
  pheadBuf = itoa(I2C1->SR1, pheadBuf,2,0);
  pheadBuf = stradd (pheadBuf, "\r\nSR2=");
  pheadBuf = itoa(I2C1->SR2, pheadBuf,2,0);
  putlog(debugSmbusBuf, pheadBuf);
#endif
  asm("nop"); // I2C1 Error
}


/* *************************************************************
This routine launch SMBus command Read_Word
it will be execute step by step in inteerrup
address       - address of slave device
command       - command to slave device
ReadWCallback - pointer to CallBack routine at success read to
return: 0 - failed start SMBus command Read_Word
        1 - successfull start.
************************************************************* */
uint8_t I2C_ReadWord(uint8_t address, uint8_t command, void (*ReadWCallback)(uint16_t *pData))
{
  if (  (smbusState == SMBUS_FREE) &&
        (!(I2C1->SR2 & I2C_SR2_BUSY)) )
  {
    uint16_t sr1 = I2C1->SR1;
    smbusState = SMBUS_READ_WORD;
    i2cState = I2C_FIRST_EV5;
    addr = address;
    comm = command;
    Callback = ReadWCallback;
    I2C1->CR1 |= I2C_CR1_START; //stage send START
    I2C1->CR1 |= I2C_CR1_ACK;
  #ifdef SWO_SMBUS_LOG
    pheadBuf = stradd (debugSmbusBuf, "\r\n");
    pheadBuf = stradd (pheadBuf, "\r\nSR1=");
    pheadBuf = itoa(sr1, pheadBuf,2,0);
    pheadBuf = stradd (pheadBuf, "\r\nSTART sent");
    putlog(debugSmbusBuf, pheadBuf);
  #endif
    return(1);
  }
  return(0);
}

    
 /* 
  while (!(I2C1->sr1 & I2C_SR1_ADDR))     // wait ADDR Reset in Slave Read sr1 SR2
    if(I2C1->sr1 & I2C_SR1_AF)           // if NACK
      while(1) {};  
  (void) I2C1->SR2;                     // clear ADDR
  
  I2C1->DR=0x07;  //read temperature
  while (!(I2C1->sr1 & I2C_SR1_TXE))        // wait BTF  Reset by Write DR
    if(I2C1->sr1 & I2C_SR1_AF)                // if NACK
      while(1) {};
  
  I2C1->CR1 |= I2C_CR1_START;           //restart
  while (!(I2C1->sr1 & I2C_SR1_SB));    // wait SB
    asm("nop");
  
  I2C1->DR = (addr<<1) | READ;
  while (!(I2C1->sr1 & I2C_SR1_ADDR))        // wait ADDR
    if(I2C1->sr1 & I2C_SR1_AF)           // if NACK
      while(1) {};
  (void) I2C1->SR2;                           // clear ADDR
  
  while (!(I2C1->sr1 & I2C_SR1_RXNE))        // wait incoming data Reset by Read DR
    if(I2C1->sr1 & I2C_SR1_AF)                // if NACK
      while(1) {};
 
 uint16_t indata = (uint8_t)I2C1->DR;
 while (!(I2C1->sr1 & I2C_SR1_RXNE))        // wait incoming data
    if(I2C1->sr1 & I2C_SR1_AF)                // if NACK
      while(1) {};
 indata |= ((uint8_t)I2C1->DR)<<8;
 I2C1->CR1 |= I2C_CR1_STOP;
 //nujno li CR1_ASK ??


void I2C1_EV_IRQHandler()
{
  uint16_t sr1 = I2C1->SR1;
#ifdef SWO_SMBUS_LOG
  pheadBuf = stradd (debugSmbusBuf, "\r\n");  //new debugging string
  pheadBuf = stradd (pheadBuf, "\r\nSR1=");
  pheadBuf = itoa(sr1, pheadBuf,2,0);
#endif
  if (smbusState == SMBUS_READ_WORD)
  {
    switch(i2cState)
    {
    case I2C_STATE_STOP:
      if(sr1 & I2C_SR1_SB)
      {
        i2cState = I2C_STATE_START;
        I2C1->DR = (addr<<1) | WRITE;
      #ifdef SWO_SMBUS_LOG
        pheadBuf = stradd (pheadBuf, "\r\nADDR | W sent");
      #endif      
      }
      break;
      
      case I2C_STATE_START:
        if (sr1 & I2C_SR1_ADDR)
        {
          (void) I2C1->SR2;
          i2cState = I2C_STATE_ADDRESS_W;
          I2C1->DR = comm;
        #ifdef SWO_SMBUS_LOG
          pheadBuf = stradd (pheadBuf, "\r\nCOMMAND sent");
        #endif
        }
        break;
       
      case I2C_STATE_ADDRESS_W:  
        if (sr1 & I2C_SR1_TXE)
        {
          i2cState = I2C_STATE_COMMAND;
          I2C1->CR1 |= I2C_CR1_START;
        #ifdef SWO_SMBUS_LOG
          pheadBuf = stradd (pheadBuf, "\r\nRESTART sent");
        #endif
        }
        break;
        
      case I2C_STATE_COMMAND:
        if(sr1 & I2C_SR1_SB)
        {
          i2cState = I2C_STATE_RESTART;
          I2C1->DR = (addr<<1) | READ;
        #ifdef SWO_SMBUS_LOG
          pheadBuf = stradd (pheadBuf, "\r\nADDR | R sent");
        #endif
        }
        break;
      
      case I2C_STATE_RESTART: 
        if (sr1 & I2C_SR1_ADDR)
        {
          (void) I2C1->SR2;
          i2cState = I2C_STATE_ADDRESS_R;
          I2C1->CR1 |= I2C_CR1_ACK;
        #ifdef SWO_SMBUS_LOG
          pheadBuf = stradd (pheadBuf, "\r\nWait low byte");
        #endif
        }
        break;
       
      case I2C_STATE_ADDRESS_R:
        if (sr1 & I2C_SR1_RXNE)
        {
          i2cState = I2C_STATE_READ_LOW;
          data = (uint8_t)I2C1->DR;
        #ifdef SWO_SMBUS_LOG
          pheadBuf = stradd (pheadBuf, "\r\nWait high byte");
        #endif  
        }
        break;
       
      case I2C_STATE_READ_LOW:
        if (sr1 & I2C_SR1_RXNE)
        {
          data |= ((uint8_t)I2C1->DR)<<8;
          I2C1->CR1 |= I2C_CR1_STOP;
        #ifdef SWO_SMBUS_LOG
          pheadBuf = stradd (pheadBuf, "\r\nSTOP sent");
        #endif
          if (Callback)
          {
            Callback();
            smbusState = SMBUS_FREE;
          }
        }
        break; 
      
    default:
      i2cState = I2C_STATE_STOP;
    } //switch(i2cState)
  }
  else if (smbusState == SMBUS_WRITE_WORD)
  {
    asm("nop");
    
    
    
  }
#ifdef SWO_SMBUS_LOG  
  putlog(debugSmbusBuf, pheadBuf);
#endif  
}// I2C1 Event


*/