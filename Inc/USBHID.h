#ifndef _USBHID_H_
 #define _USBHID_H_
 #include "stm32f10x.h"
 #include "stm32f103xb_usbdef.h"
 void USB_LP_CAN1_RX0_IRQHandler();
 void USB_Reset();
 void USB_EPHandler(uint16_t Status);
 void USBLIB_Pma2EPBuf(uint8_t EPn);
 void USBLIB_GetDescriptor(USBLIB_SetupPacket *SPacket);
 void USBLIB_SendData(uint8_t EPn, uint16_t *Data, uint16_t Length);
 void USBLIB_EPBuf2Pma(uint8_t EPn);
 void USBLIB_setStatTx(uint8_t EPn, uint16_t Stat);
 void USBLIB_setStatRx(uint8_t EPn, uint16_t Stat);
 
 void loggingSetupPacket(USBLIB_SetupPacket *pSetup);
#endif //_USBHID_H_