#include "Systick.h"
#include <stdlib.h>

Typedef_Stimer_inf *pStimers;
uint8_t timersTotal = 0;
volatile uint32_t SysTickCounter;
uint32_t ticks=0;

void SysTick_Handler()
{
  ticks++;
  if(SysTickCounter>0)
    SysTickCounter--;
  for(uint8_t i=0;i<timersTotal;i++)
  {
    Typedef_Stimer_inf *pTimer = pStimers + i;
    if (pTimer->state)
    {
      if(--(pTimer->value) == 0)
      {
        pTimer->value = pTimer->interval;
        pTimer->cycleCnt++;
        pTimer->stimer_Handler();
      }
    }
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

uint8_t SysTick_TimersCreate(uint8_t total)
{
  pStimers = 0;
  if (!timersTotal)
  {
    pStimers = (Typedef_Stimer_inf*) malloc (total*sizeof(Typedef_Stimer_inf));
    if (pStimers)
    {
      timersTotal = total;
      for(uint8_t i=0;i<total;i++)
      {
        Typedef_Stimer_inf *pTimer = pStimers + i;
        pTimer->state = 0;
        pTimer->stimer_Handler = NULL;
      }
      return (1);
    }
  }
  return (0);
}

uint8_t SysTick_TimerInit(uint8_t num, void (*stimer_callback)(void))
{
  if ((pStimers) && (num < timersTotal))
  {
    Typedef_Stimer_inf *pTimer = pStimers + num;
    if (pTimer->state == 0)
    {
      pTimer->cycleCnt = 0;
      pTimer->state = 0;
      pTimer->stimer_Handler = stimer_callback;
      pTimer->value = 0;
      return (1);
    }
  }
  return (0); 
}

uint8_t SysTick_TimerRun(uint8_t num, uint32_t interval)
{
  if ((pStimers) && (num < timersTotal))
  {
    Typedef_Stimer_inf *pTimer = pStimers + num;
    if (pTimer->state == 0)
    {
      pTimer->cycleCnt = 0;
      pTimer->interval = interval;
      pTimer->value = interval;
      pTimer->state = 1;
      return (1);
    }
  }
  return (0); 
}

uint8_t SysTick_TimerStop(uint8_t num)
{
  if ((pStimers) && (num < timersTotal))
  {
    Typedef_Stimer_inf *pTimer = pStimers + num;
    pTimer->state = 0;
    return (1);
  }
  return (0); 
}