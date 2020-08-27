//SYSCLK=48MHz
#include "stm32f103xb_usbdef.h"
#include "cll_stm32F10x_gpio.h"
#include "Systick.h"
#include "additional_func.h"
#include "oringbuf.h"
//for SWO logging activate SWOLOG in Options\C++ compiler\Defined Symbol

#define SYSTICK_DIVIDER 72000
//GPIO A
#define USB_ENABLE 7
#define USB_DM 11
#define USB_DP 12
//GPIO C
#define ONBOARD_LED 13

#ifdef SWOLOG
   uint8_t toSWO;
#endif

void main()
{

  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN //GPIO C enable
    | RCC_APB2ENR_IOPAEN    //GPIO A
    | RCC_APB2ENR_AFIOEN; // Alternate function enable
  
  PinParametr GPIO_descript[3];
  GPIO_descript[0].PinPos=USB_DM;
  GPIO_descript[0].PinMode=GPIO_MODE_OUTPUT50_ALT_PUSH_PULL;
  
  GPIO_descript[1].PinPos=USB_DP;
  GPIO_descript[1].PinMode=GPIO_MODE_OUTPUT50_ALT_PUSH_PULL;
  
  GPIO_descript[2].PinPos=USB_ENABLE;
  GPIO_descript[2].PinMode=GPIO_MODE_OUTPUT50_OPEN_DRAIN;
  CLL_GPIO_SetPinMode(GPIOA,GPIO_descript,3);   //GPIO A init
  GPIO_SET(GPIOA,1<<USB_ENABLE); //Disable USB pullup resistor 1k5
  
  GPIO_descript[0].PinPos=ONBOARD_LED;
  GPIO_descript[0].PinMode=GPIO_MODE_OUTPUT50_PUSH_PULL;  
  CLL_GPIO_SetPinMode(GPIOC,GPIO_descript,1);   //GPIO C init
  GPIO_SET(GPIOC,1<<ONBOARD_LED); //off LED 
  
#ifdef SWOLOG
   //SWO debug ON
  DBGMCU->CR &= ~(DBGMCU_CR_TRACE_MODE_0 | DBGMCU_CR_TRACE_MODE_0);
  DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;
  // JTAG-DP Disabled and SW-DP Enabled
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;
  
  if (!Oringbuf_Create(8100))
    while(1)
      asm("nop");   //don't have memory maybe
#endif
  
  //1ms SysTick interval.
  while (SysTick_Config(SYSTICK_DIVIDER)==1)	
    asm("nop");	 //reason - bad divider 
  NVIC_EnableIRQ(SysTick_IRQn);
  
  //Turn ON the takt core couner
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;// Enable DWT
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; //counter ON
  DWT->CYCCNT = 0; //start value = 0
  
 //USB initialize
  GPIO_RESET(GPIOA,1<<USB_ENABLE); //Enable USB pullup resistor 1k5
  //RCC->CFGR |= RCC_CFGR_USBPRE; //not divide PLL clock for USB 48MHz
  RCC->APB1ENR |= RCC_APB1ENR_USBEN; //clocking USB Enable
  USB->CNTR &= ~USB_CNTR_PDWN; //disable PowerDown
  msDelay(1);
  USB->ISTR =0;     //reset all pending interrupts
  USB->BTABLE = 0;  
  USB->CNTR |= USB_CNTR_RESETM;  //USB reset interrupt enable  
  USB->CNTR &= ~USB_CNTR_FRES; //disable Force Reset
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  NVIC_EnableIRQ(USBWakeUp_IRQn);
  //msDelay(4000);
  //GPIO_SET(GPIOA,1<<USB_ENABLE); //Disable USB pullup resistor 1k5
  while (1)
  {
  if(Oringbuf_Get(&toSWO,1))
    ITM_SendChar((uint32_t) toSWO);
  }
}

