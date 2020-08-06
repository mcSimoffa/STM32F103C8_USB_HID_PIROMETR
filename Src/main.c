//SYSCLK=48MHz
#include "stm32f103xb_usbdef.h"
#include "cll_stm32F10x_gpio.h"
#include "Systick.h"
#define SYSTICK_DIVIDER 48000
//GPIO A
#define USB_DM 11
#define USB_DP 12
//GPIO C
#define ONBOARD_LED 13

void main()
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN //GPIO C enable
    | RCC_APB2ENR_IOPAEN    //GPIO A
    | RCC_APB2ENR_AFIOEN; // Alternate function enable
  
  PinParametr GPIO_descript[2];
  GPIO_descript[0].PinPos=USB_DM;
  GPIO_descript[0].PinMode=GPIO_MODE_OUTPUT50_ALT_PUSH_PULL;
  
  GPIO_descript[1].PinPos=USB_DP;
  GPIO_descript[1].PinMode=GPIO_MODE_OUTPUT50_ALT_PUSH_PULL;
  CLL_GPIO_SetPinMode(GPIOA,GPIO_descript,2);   //GPIO A init
  
  GPIO_descript[0].PinPos=ONBOARD_LED;
  GPIO_descript[0].PinMode=GPIO_MODE_OUTPUT50_PUSH_PULL;  
  CLL_GPIO_SetPinMode(GPIOC,GPIO_descript,1);   //GPIO C init
  GPIO_SET(GPIOC,1<<ONBOARD_LED); //off LED 
  
  //1ms SysTick interval.
  while (SysTick_Config(SYSTICK_DIVIDER)==1)	
    asm("nop");	 //reason - bad divider 
  NVIC_EnableIRQ(SysTick_IRQn);
 
  RCC->CFGR |= RCC_CFGR_USBPRE; //not divide PLL clock for USB 48MHz
  RCC->APB1ENR |= RCC_APB1ENR_USBEN; //clocking USB Enable
  USB->CNTR &= ~USB_CNTR_PDWN;
  msDelay(1);
  //USB->CNTR &= USB_CNTR_FRES; //disable PowerDown
  USB->ISTR =0;     //reset all interrupt flags
  USB->BTABLE = 0;
  USB->CNTR |= USB_CNTR_CTRM    //correct transfer interrupt enable
    | USB_CNTR_PMAOVRM  //packet memory area over/under interrupt enable
    | USB_CNTR_ERRM     //error interrupt enable
    | USB_CNTR_WKUPM    //wake up  interrupt enable
    | USB_CNTR_SUSPM    //suspend mode interrupt enable
    | USB_CNTR_RESETM   //USB reset interrupt enable  
    | USB_CNTR_SOFM     //Start of frame interrupt enable  
    | USB_CNTR_ESOFM;   //Expected start of frame interrupt enable 
  __enable_irq ();
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  NVIC_EnableIRQ(USBWakeUp_IRQn);
  while (1)
  {
  }
}

