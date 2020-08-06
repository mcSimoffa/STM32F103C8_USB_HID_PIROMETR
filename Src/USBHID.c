#include "USBHID.h"

void USB_LP_CAN1_RX0_IRQHandler()
{
while(1)
{
  GPIO_RESET(GPIOC,1<<ONBOARD_LED); //on LED 
  for(uint32_t i=1000000;i>0;i--)
  {
  }
  GPIO_SET(GPIOC,1<<ONBOARD_LED); //on LED 
  for(uint32_t i=1000000;i>0;i--)
  {
  }
}
}