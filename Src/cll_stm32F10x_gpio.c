#include "stm32f10x.h"
#include "cll_stm32F10x_gpio.h"


void CLL_GPIO_SetPinMode(GPIO_TypeDef *GPIOx,  const PinParametr *Mode, const char ElemCount)
{
  unsigned long long Accum;
  Accum =((unsigned long long)(GPIOx->CRH)<<32);
  Accum |= ((unsigned long long)(GPIOx->CRL));
  for (int32_t i=ElemCount-1;i>=0;i--)
  {
    Accum &= ~(unsigned long long)((unsigned long long)(0x0F)<<(Mode[i].PinPos<<2));
    Accum |=(unsigned long long)((unsigned long long)(Mode[i].PinMode)<<(Mode[i].PinPos<<2));
  } 
  GPIOx->CRH=(uint32_t)((unsigned long long)(Accum>>32));
  GPIOx->CRL=(uint32_t)((unsigned long long)(Accum));
 return; 
}

void CLL_GPIO_SetOnePinMode(GPIO_TypeDef *GPIOx,  const uint8_t Pin_Num, GPIOMode_TypeDef PinParametr)
{
  uint32_t Accum;  
  Accum =*(uint32_t *)((uint32_t *)GPIOx+(Pin_Num>>3));
  Accum &= ~(uint32_t)((uint32_t)(0x0F)<<((Pin_Num & ~8)<<2));
  Accum |=(uint32_t)((uint32_t)(PinParametr)<<((Pin_Num & ~8)<<2));
  *(uint32_t *)((uint32_t *)GPIOx+(Pin_Num>>3)) = Accum;
 return; 
}

