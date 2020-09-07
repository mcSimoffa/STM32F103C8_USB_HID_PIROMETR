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
/* ****************************************************************************
This routine delayed thread
delay - time in tick to delayed
**************************************************************************** */
void msDelay(uint32_t delay)
{
  SysTick->VAL = 0;
  SysTickCounter=delay;
  do
    asm("nop");
  while (SysTickCounter>0);
   asm("nop"); 
}

/* ****************************************************************************
This routine return the time since Systick started 
**************************************************************************** */
uint32_t GetTick()
{
 return(ticks);
}

/* ****************************************************************************
This routine creates several independed timers based on Systick interval
total - quantity of independed timers
Note: each independed timer  require 20 bytes in HEAP
return value: 1 - if success or 0 - if Fail
reasons Fail: no enough memmort, aleready have independed timers
**************************************************************************** */
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

/* ****************************************************************************
This routine assign Callback function for one independed timer 
num             - number of independed timer
stimer_callback - pointer to Callback function
return value: 1 - if success or 0 - if Fail
reasons Fail: haven't independed timers, num > total quantity timers, timer aleready Run
**************************************************************************** */
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

/* ****************************************************************************
This routine Run independed timer with given interval
num       - number of independed timer
interval  - interval
return value: 1 - if success or 0 - if Fail
reasons Fail: haven't independed timers, num > total quantity timers,  timer aleready Run
**************************************************************************** */
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

/* ****************************************************************************
This routine Stop independed timer
num       - number of independed timer
return value: 1 - if success or 0 - if Fail
reasons Fail: haven't independed timers, num > total quantity timers
**************************************************************************** */
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