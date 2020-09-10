#include "i2c.h"
I2C_STATES state = I2C_STATE_STOP;
uint8_t addr;
uint8_t comm;
uint16_t data;
void I2C1_EV_IRQHandler()
{
  switch(state)
  {
  case I2C_STATE_STOP:
    if(I2C1->SR1 & I2C_SR1_SB)
    {
      state = I2C_STATE_START;
      I2C1->DR = (addr<<1) | WRITE; //Stage send ADDRESS
    }
    break;
    
    case I2C_STATE_START:
      if (I2C1->SR1 & I2C_SR1_ADDR)
      {
       state = I2C_STATE_ADDRESS_W;
       I2C1->DR = comm; //Stage send COMMAND
      }
      break;
     
    case I2C_STATE_ADDRESS_W:  
      if (I2C1->SR1 & I2C_SR1_TXE)
      {
        state = I2C_STATE_COMMAND;
        I2C1->CR1 |= I2C_CR1_START; //Stage send RESTART
      }
      break;
      
    case I2C_STATE_COMMAND:
      if(I2C1->SR1 & I2C_SR1_SB)
      {
        state = I2C_STATE_RESTART;
        I2C1->DR = (addr<<1) | READ; //Stage send ADDRESS after RESTART
      }
      break;
    
    case I2C_STATE_RESTART: 
      if (I2C1->SR1 & I2C_SR1_ADDR)
      {
       state = I2C_STATE_ADDRESS_R;
       I2C1->DR = comm; //Stage send COMMAND
      }
      break;
     
    case I2C_STATE_ADDRESS_R:
      if (I2C1->SR1 & I2C_SR1_RXNE)
      {
        state = I2C_STATE_READ_LOW;
        data = (uint8_t)I2C1->DR;
      }
      break;
     
    case I2C_STATE_READ_LOW:
      if (I2C1->SR1 & I2C_SR1_RXNE)
      {
        data |= ((uint8_t)I2C1->DR)<<8;
        I2C1->CR1 |= I2C_CR1_STOP;
      }
      break; 
    
  default:
    state = I2C_STATE_STOP;
  } //switch(state)   
}// I2C1 Event

void I2C1_ER_IRQHandler()
{
}// I2C1 Error

uint8_t I2C_ReadWord(uint8_t address, uint8_t command, void (*DoneCallback)(void))
{
  if (I2C1->SR2 & I2C_SR2_BUSY)
    return(0);
  addr = address;
  comm = command;
  state = I2C_STATE_STOP;
  I2C1->CR1 |= I2C_CR1_START; //stage send START
  return(1);
}

    
 /* 
  while (!(I2C1->SR1 & I2C_SR1_ADDR))     // wait ADDR Reset in Slave Read SR1 SR2
    if(I2C1->SR1 & I2C_SR1_AF)           // if NACK
      while(1) {};  
  (void) I2C1->SR2;                     // clear ADDR
  
  I2C1->DR=0x07;  //read temperature
  while (!(I2C1->SR1 & I2C_SR1_TXE))        // wait BTF  Reset by Write DR
    if(I2C1->SR1 & I2C_SR1_AF)                // if NACK
      while(1) {};
  
  I2C1->CR1 |= I2C_CR1_START;           //restart
  while (!(I2C1->SR1 & I2C_SR1_SB));    // wait SB
    asm("nop");
  
  I2C1->DR = (addr<<1) | READ;
  while (!(I2C1->SR1 & I2C_SR1_ADDR))        // wait ADDR
    if(I2C1->SR1 & I2C_SR1_AF)           // if NACK
      while(1) {};
  (void) I2C1->SR2;                           // clear ADDR
  
  while (!(I2C1->SR1 & I2C_SR1_RXNE))        // wait incoming data Reset by Read DR
    if(I2C1->SR1 & I2C_SR1_AF)                // if NACK
      while(1) {};
 
 uint16_t indata = (uint8_t)I2C1->DR;
 while (!(I2C1->SR1 & I2C_SR1_RXNE))        // wait incoming data
    if(I2C1->SR1 & I2C_SR1_AF)                // if NACK
      while(1) {};
 indata |= ((uint8_t)I2C1->DR)<<8;
 I2C1->CR1 |= I2C_CR1_STOP;
 //nujno li CR1_ASK ??
*/