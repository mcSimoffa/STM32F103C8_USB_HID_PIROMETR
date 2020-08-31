#include "Systick.h"

Typedef_SystickCallback systickSheduler={
.interval = 0,    //interval in msec
.body = 0         //CallBack routine Executed every interval
};
volatile uint32_t SysTickCounter;
uint32_t ticks=0;

void SysTick_Handler()
{
  static uint32_t shedulerTick=1;
  ticks++;
  if(SysTickCounter>0)
    SysTickCounter--;
  if(systickSheduler.body)
    if (--shedulerTick == 0)
    {
     shedulerTick = systickSheduler.interval;
     systickSheduler.body();
    }
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

uint32_t GetTick()
{
 return(ticks);
}
  

