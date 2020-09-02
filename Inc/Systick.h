#ifndef _SYSTICK_H_
 #define _SYSTICK_H_
 #include "stm32f10x.h"
 typedef struct
 {
   uint32_t interval;
   void (*body)(void);
 } Typedef_SystickCallback;

 void SysTick_Handler();
 void msDelay(uint32_t delay);
 uint32_t GetTick();
#endif //_SYSTICK_H_