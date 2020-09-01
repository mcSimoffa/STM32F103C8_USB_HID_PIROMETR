#ifndef _USBHID_H_
 #define _USBHID_H_
 #include "stm32f10x.h"
 #include "stm32f103xb_usbdef.h"

//EP Send state:
#define EP_SEND_BUSY       0
#define EP_SEND_INITIATE   1
#define EP_SEND_READY      2

typedef struct 
{
    uint16_t  Number;      // EP number
    uint16_t  Type;        // EP Type
    uint8_t   TX_Max;      // Max TX EP Buffer
    uint8_t   RX_Max;      // Max RT EP Buffer
    uint16_t *pTX_BUFF;    // TX Buffer pointer
    uint32_t  lTX;         // TX Data length
    uint16_t *pRX_BUFF;    // RX Buffer pointer
    uint32_t  lRX;         // RX Data length
    uint8_t  SendState;    //state of Send mashine
} USB_EPinfo;

typedef struct
{
  uint16_t cnt;       //counter reading bytes
  uint16_t totalSize; //expected size of transaction inclusiv Setup
  uint8_t *pData;     //pointer to Buffer accumulating all data packets for Setup transaction
  uint16_t sizeData;  //size of this Buffer (use malloc) 
} Typedef_OUT_TransactionCollector;

 void USB_LP_CAN1_RX0_IRQHandler();
 void USB_Reset();
 void USB_EPHandler(uint16_t Status);
 void USBLIB_Pma2EPBuf(uint8_t EPn);
 void USBLIB_SendData(uint8_t EPn, uint16_t *Data, uint16_t Length);
 void USBLIB_EPBuf2Pma(uint8_t EPn);
 void USBLIB_setStatTx(uint8_t EPn, uint16_t Stat);
 void USBLIB_setStatRx(uint8_t EPn, uint16_t Stat);
 uint8_t USB_sendReport(uint8_t EPn, uint16_t *Data, uint16_t Length);
#endif //_USBHID_H_