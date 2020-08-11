#ifndef _SYSTICK_H_
 #define SYSTICK_H_
 #include "stm32f10x.h"
 void SysTick_Handler();
 void msDelay(uint32_t delay);
 uint32_t GetTick();
#endif //_SYSTICK_H_