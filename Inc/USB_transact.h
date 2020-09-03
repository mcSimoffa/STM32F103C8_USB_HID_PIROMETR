#ifndef _USB_TRANSACT_H_
 #define _USB_TRANSACT_H_
 #include "stm32f10x.h"

  uint8_t   USB_IN_requestHandler(USB_SetupPacket *pSetup, uint16_t **ppDataToIn, uint16_t *sizeData);
  uint8_t USB_OUT_requestHandler(USB_SetupPacket *pSetup, uint16_t *pDataOut, uint16_t sizeData);
  void      USB_setDeviceCofig(uint16_t val);
  uint16_t  USB_getDeviceConfig();
#endif //_USB_TRANSACT_H_