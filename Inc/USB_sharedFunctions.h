#ifndef _USB_SHAREDFUNCTIONS_H_
 #define _USB_SHAREDFUNCTIONS_H_
  #include "stm32f10x.h"

  void      USB_setAddress(uint8_t address);
  uint16_t  USB_geStatusEP(uint8_t EPnum);
  //uint8_t USB_getAddress();
#endif //_USB_SHAREDFUNCTIONS_H_
