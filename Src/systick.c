#include "Systick.h"

volatile uint32_t SysTickCounter;
void SysTick_Handler()
{
  if(SysTickCounter>0)
    SysTickCounter--;
}
 
void msDelay(uint32_t delay)
{
  SysTick->VAL = 0;
  SysTickCounter=delay;
  do
    asm("nop");
  while (SysTickCounter>0);
   asm("nop"); 
}
