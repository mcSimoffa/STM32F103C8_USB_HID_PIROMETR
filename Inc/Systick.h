#ifndef _SYSTICK_H_
 #define _SYSTICK_H_
 #include "stm32f10x.h"

 typedef struct
 {
   uint32_t interval;
   uint32_t value;
   uint32_t cycleCnt;
   void (*stimer_Handler)(void);
   uint8_t state;
 }Typedef_Stimer_inf;

 void SysTick_Handler();
 void msDelay(uint32_t delay);
 uint32_t GetTick();
 uint8_t SysTick_TimersCreate(uint8_t total);
 uint8_t SysTick_TimerInit(uint8_t num, void (*stimer_callback)(void));
 uint8_t SysTick_TimerRun(uint8_t num, uint32_t interval);
 uint8_t SysTick_TimerStop(uint8_t num);
#endif //_SYSTICK_H_