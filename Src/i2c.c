#include "i2c.h"
#include "SWOfunction.h"
#include "additional_func.h"

I2C_STATES    i2cState = I2C_STATE_FREE;
SMBUS_STATES  smbusState = SMBUS_FREE; 
uint8_t addr; //Address I2C Slave Device
uint8_t comm; //command for lave Device
uint16_t data;
void (*Callback)(uint16_t *pData);

#ifdef SWO_SMBUS_LOG
  char debugSmbusBuf[256];
  char *pheadBuf;
#endif

/* *************************************************************
This routine first copy data from register to storage
next execute software reset
next it copy storaged data back
************************************************************* */ 
void I2C_Reinit()
{
  uint16_t copy_CR1=(I2C1->CR1) & ~I2C_CR1_PE & ~I2C_CR1_START & ~I2C_CR1_STOP;
  uint16_t copy_CR2 = I2C1->CR2;
  uint16_t copy_CCR = I2C1->CCR;
  uint16_t copy_TRISE = I2C1->TRISE;
  I2C1->CR1 |= I2C_CR1_SWRST; //Reset I2C
  asm("nop");
  I2C1->CR1 &= ~I2C_CR1_SWRST;
  I2C1->TRISE = copy_TRISE;
  I2C1->CCR = copy_CCR;
  I2C1->CR2 = copy_CR2;
  I2C1->CR1 = copy_CR1;
}
  
/* *************************************************************
I2C1 interrupt Handler
************************************************************* */  
void I2C1_EV_IRQHandler()
{
  uint16_t sr1 = I2C1->SR1;
  uint16_t sr2 = I2C1->SR2;
#ifdef SWO_SMBUS_LOG
  pheadBuf = stradd (debugSmbusBuf, "\r\n\nTAKT # ");
  pheadBuf = itoa(DWT->CYCCNT, pheadBuf,10,0);
  pheadBuf = stradd (pheadBuf, "\r\nSR1=");
  pheadBuf = itoa(sr1, pheadBuf,2,0);
  pheadBuf = stradd (pheadBuf, "\r\nSR2=");
  pheadBuf = itoa(sr2, pheadBuf,2,0);
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
          //(void) I2C1->SR2;
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
          I2C1->CR2 &= ~I2C_CR2_ITBUFEN;
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
          //(void) I2C1->SR2;
          i2cState = I2C_STATE_DATAWAIT;
          I2C1->CR1 &= ~I2C_CR1_ACK;
        #ifdef SWO_SMBUS_LOG
          pheadBuf = stradd (pheadBuf, "\r\nWait Data NACK");
        #endif
        }
        break;
       
      case I2C_STATE_DATAWAIT:
        if (sr1 & I2C_SR1_BTF)
        {
          i2cState = I2C_STATE_FREE;
          smbusState = SMBUS_FREE;
          I2C1->CR1 |= I2C_CR1_STOP;
          data =  (uint8_t)I2C1->DR;
          data |= ((uint8_t)I2C1->DR) << 8;
          if (Callback)
            Callback(&data);
        #ifdef SWO_SMBUS_LOG
          pheadBuf = stradd (pheadBuf, "\r\nStop sent,wData=");
          pheadBuf = itoa(data, pheadBuf,16,0);
        #endif  
        }
        break;
      
    default:
      i2cState = I2C_STATE_FREE;
    } //switch(i2cState)
  }
  else if (smbusState == SMBUS_WRITE_WORD)
  {
    asm("nop"); //TODO
  }
   if (sr1 & I2C_SR1_STOPF) //it's troble situation for Master (it became Slave). It Occur when line SCL SDA broken or short circuit have 
   {
    I2C_Reinit();
    i2cState = I2C_STATE_FREE;
    smbusState = SMBUS_FREE;
   }
#ifdef SWO_SMBUS_LOG  
  putlog(debugSmbusBuf, pheadBuf);
#endif  
}// I2C1 Event

/* *************************************************************
I2C1 Errror interrupt Handler
if errors occur this routine execute Software Reset
and change state machine to ready for new command
************************************************************* */
void I2C1_ER_IRQHandler()
{
#ifdef SWO_SMBUS_LOG
  pheadBuf = stradd (debugSmbusBuf, "\r\nERROR");
  pheadBuf = stradd (pheadBuf, "\r\nSR1=");
  pheadBuf = itoa(I2C1->SR1, pheadBuf,2,0);
  pheadBuf = stradd (pheadBuf, "\r\nSR2=");
  pheadBuf = itoa(I2C1->SR2, pheadBuf,2,0);
  pheadBuf = stradd (pheadBuf, "\r\nError! Do reinit"); 
#endif
  I2C_Reinit();
#ifdef SWO_SMBUS_LOG
  pheadBuf = stradd (pheadBuf, "\r\nSR1=");
  pheadBuf = itoa(I2C1->SR1, pheadBuf,2,0);
  pheadBuf = stradd (pheadBuf, "\r\nSR2=");
  pheadBuf = itoa(I2C1->SR2, pheadBuf,2,0);
  putlog(debugSmbusBuf, pheadBuf);
#endif
  i2cState = I2C_STATE_FREE;
  smbusState = SMBUS_FREE;
  
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
  if (!(I2C1->CR1 & I2C_CR1_PE))
  {
    I2C1->CR1  |= I2C_CR1_PE;
    return(0);
  }
  uint16_t sr1 = I2C1->SR1;
  uint16_t sr2 = I2C1->SR2;
  //master executed STOP command and clear STOP bit 
  //& machine state ready to new command
  if ( (smbusState == SMBUS_FREE) && (!(I2C1->CR1 & I2C_CR1_STOP)) )
  {
    if (sr2 & I2C_SR2_BUSY) //it's troble situation: have permanent BUSY bit
    {
      I2C_Reinit();
      return(0);
    }
  #ifdef SWO_SMBUS_LOG
    DWT->CYCCNT = 0;
    pheadBuf = stradd (debugSmbusBuf, "\r\n\n\n----- TAKT # ");
    pheadBuf = itoa(DWT->CYCCNT, pheadBuf,10,0);
    pheadBuf = stradd (pheadBuf, "\r\nSR1=");
    pheadBuf = itoa(sr1, pheadBuf,2,0);
    pheadBuf = stradd (pheadBuf, "\r\nSR2=");
    pheadBuf = itoa(sr2, pheadBuf,2,0);
    pheadBuf = stradd (pheadBuf, "\r\nSTART sent");
    putlog(debugSmbusBuf, pheadBuf);
  #endif
    smbusState = SMBUS_READ_WORD;
    i2cState = I2C_FIRST_EV5;
    addr = address;
    comm = command;
    Callback = ReadWCallback;
    I2C1->CR1 |= I2C_CR1_START; //stage send START
    I2C1->CR1 |= I2C_CR1_ACK;
    return(1);
  }
  return(0);
}
