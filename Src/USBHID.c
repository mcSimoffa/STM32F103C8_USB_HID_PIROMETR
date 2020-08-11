#include "USBHID.h"
#include <stdlib.h>
#include "Systick.h"
#define EPCOUNT 2
__no_init volatile USB_BDT BDTable[EPCOUNT] @ USB_PMAADDR ; //Buffer Description Table


typedef struct
{
  uint32_t tick_;
  char com_1;
  char com_2;
  uint16_t reg_16;
  uint32_t reg_32;
} Typedef_debug ;

Typedef_debug debug[100];
void writeDebug(char com1, char com2,uint16_t reg16, uint32_t reg32);

USB_EPinfo EpData[EPCOUNT] =
{
  {.Number=0, .Type=EP_CONTROL,     .TX_Max=8, .RX_Max=8, .pTX_BUFF=0, .lTX=0, .pRX_BUFF=0, .lRX=0},
  {.Number=1, .Type=EP_INTERRUPT,   .TX_Max=8, .RX_Max=8, .pTX_BUFF=0, .lTX=0, .pRX_BUFF=0, .lRX=0}
};
USBLIB_SetupPacket   *SetupPacket;
volatile uint8_t      DeviceAddress = 0;

const uint8_t USB_DEVICE_DESC[] =
    {
        (uint8_t)18,                        //    bLength
        (uint8_t)USB_DEVICE_DESC_TYPE,      //    bDescriptorType
        (uint8_t)0x00,                      //    bcdUSB
        (uint8_t)0x02,                      //    bcdUSB
        (uint8_t)USB_COMM,                  //    bDeviceClass
        (uint8_t)0,                         //    bDeviceSubClass
        (uint8_t)0,                         //    bDeviceProtocol
        (uint8_t)8,                         //    bMaxPacketSize0
        (uint8_t)LOBYTE(DEVICE_VENDOR_ID),  //    idVendor
        (uint8_t)HIBYTE(DEVICE_VENDOR_ID),  //    idVendor
        (uint8_t)LOBYTE(DEVICE_PRODUCT_ID), //    idProduct
        (uint8_t)HIBYTE(DEVICE_PRODUCT_ID), //    idProduct
        (uint8_t)0x00,                      //    bcdDevice
        (uint8_t)0x01,                      //    bcdDevice
        (uint8_t)1,                         //    iManufacturer
        (uint8_t)2,                         //    iProduct
        (uint8_t)3,                         //    iSerialNumbert
        (uint8_t)1                          //    bNumConfigurations
};

void USB_Reset(void)
{
    uint16_t Addr = sizeof(BDTable)>>1;
    for (uint8_t i = 0; i < EPCOUNT; i++) //active endpoints
	{
        BDTable[i].TX_Address = Addr;
        BDTable[i].TX_Count   = 0;
        Addr += EpData[i].TX_Max;
        BDTable[i].RX_Address = Addr;
        if (EpData[i].RX_Max >= 64)
            BDTable[i].RX_Count = (1<<USB_COUNT_RX_BLSIZE_Pos) | ((EpData[i].RX_Max / 64) << USB_COUNT_RX_NUM_BLOCK_Pos);
        else
            BDTable[i].RX_Count = ((EpData[i].RX_Max / 2) << USB_COUNT_RX_NUM_BLOCK_Pos);

        Addr += EpData[i].RX_Max;
        if (!EpData[i].pRX_BUFF)
            EpData[i].pRX_BUFF = (uint16_t *)malloc(EpData[i].RX_Max);

       USB->EPR[i] = (EpData[i].Number | EpData[i].Type | RX_VALID | TX_NAK);
    }

    for (uint8_t i = EPCOUNT; i < 8; i++) //inactive endpoints
        USB->EPR[i] = i | RX_NAK | TX_NAK;

    USB->CNTR   = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM;
    USB->ISTR   = 0x00;
    USB->BTABLE = 0x00;
    USB->DADDR  = USB_DADDR_EF; //Enable USB device
}

void USB_LP_CAN1_RX0_IRQHandler()
{
    if (USB->ISTR & USB_ISTR_RESET) 
    { // Reset
        writeDebug('R', 'e', USB->ISTR,0);
        USB->ISTR &= ~USB_ISTR_RESET;
        USB_Reset();
        return;
    }
    if (USB->ISTR & USB_ISTR_CTR) 
    { //Handle data on EP
        USB_EPHandler((uint16_t)USB->ISTR);
        USB->ISTR &= ~USB_ISTR_CTR;
        return;
    }
    if (USB->ISTR & USB_ISTR_PMAOVR) 
    {
        USB->ISTR &= ~USB_ISTR_PMAOVR;
        // Handle PMAOVR status
        return;
    }
    if (USB->ISTR & USB_ISTR_SUSP) 
    {
        USB->ISTR &= ~USB_ISTR_SUSP;
        if (USB->DADDR & 0x7f) 
        {
            USB->DADDR = 0;
            USB->CNTR &= ~ USB_CNTR_SUSPM;
        }
        return;
    }
    if (USB->ISTR & USB_ISTR_ERR) 
    {
        USB->ISTR &= ~USB_ISTR_ERR;
        // Handle Error
        return;
    }
    if (USB->ISTR & USB_ISTR_WKUP) 
    {
        USB->ISTR &= ~USB_ISTR_WKUP;
        // Handle Wakeup
        return;
    }
    if (USB->ISTR & USB_ISTR_SOF) 
    {
        USB->ISTR &= ~USB_ISTR_SOF;
        // Handle SOF
        return;
    }
    if (USB->ISTR & USB_ISTR_ESOF) 
    {
        USB->ISTR &= ~USB_ISTR_ESOF;
        // Handle ESOF
        return;
    }
    USB->ISTR = 0;
}

void USB_EPHandler(uint16_t Status)
{
    uint16_t DeviceConfigured = 0, DeviceStatus = 0;
    uint8_t  EPn = Status & ISTR_EP_ID; //endpoint number where occured
    uint32_t EP  = USB->EPR[EPn];
    if (EP & EP_CTR_RX)     //something received ?
    { 
        USBLIB_Pma2EPBuf(EPn);
        if (EPn == 0)       //Control endpoint ?
        { 
            if (EP & USB_EP0R_SETUP) 
            {
                SetupPacket = (USBLIB_SetupPacket *)EpData[EPn].pRX_BUFF;
                switch (SetupPacket->bRequest) 
                {
                case USB_REQUEST_GET_DESCRIPTOR:
                    USBLIB_GetDescriptor(SetupPacket);
                    break;
                    
                case USB_REQUEST_SET_ADDRESS:
                    writeDebug('S', 'a', SetupPacket->wValue.L,0);
                    USBLIB_SendData(0, 0, 0);
                    DeviceAddress = SetupPacket->wValue.L;
                    break;

                case USB_REQUEST_GET_STATUS:
                    writeDebug('G', 's', 0,0);
                    //USBLIB_SendData(0, &DeviceStatus, 2);
                    break;

                case USB_REQUEST_GET_CONFIGURATION:
                    writeDebug('G', 'c', 0,0);
                    //USBLIB_SendData(0, &DeviceConfigured, 1);
                    break;

                case USB_REQUEST_SET_CONFIGURATION:
                    writeDebug('S', 'c', 0,0);
                    DeviceConfigured = 1;
                    USBLIB_SendData(0, 0, 0);
                    break;

                /*case USB_DEVICE_CDC_REQUEST_SET_COMM_FEATURE:
                    //TODO
                    break;

                case USB_DEVICE_CDC_REQUEST_SET_LINE_CODING:        //0x20
                    USBLIB_SendData(0, 0, 0);
                    break;

                case USB_DEVICE_CDC_REQUEST_GET_LINE_CODING:        //0x21
                    //SBLIB_SendData(EPn, (uint16_t *)&lineCoding, sizeof(lineCoding));
                    break;

                case USB_DEVICE_CDC_REQUEST_SET_CONTROL_LINE_STATE:         //0x22
                    LineState = SetupPacket->wValue;
                    USBLIB_SendData(0, 0, 0);
                    //uUSBLIB_LineStateHandler(SetupPacket->wValue);
                    break;*/
                }
            }
        } 
        else 
        { // Got data from another EP
            // Call user function
         //   uUSBLIB_DataReceivedHandler(EpData[EPn].pRX_BUFF, EpData[EPn].lRX);
        }
        
        USB->EPR[EPn] &= 0x78f; //reset flag CTR_RX
        USBLIB_setStatRx(EPn, RX_VALID);
    }
    
    if (EP & EP_CTR_TX) //something transmitted
      { 
        if (DeviceAddress) 
        {
            USB->DADDR    = DeviceAddress | 0x80;
            DeviceAddress = 0;
        }

        if (EpData[EPn].lTX) //Have to transmit something?
        {           
            USBLIB_EPBuf2Pma(EPn);
            USBLIB_setStatTx(EPn, TX_VALID);
        } else 
          {
            //uUSBLIB_DataTransmitedHandler(EPn, EpData[EPn]);
          }

        USB->EPR[EPn] &= 0x870f;   //reset flag CTR_TX
      }
}

void USBLIB_GetDescriptor(USBLIB_SetupPacket *SPacket)
{
    uint8_t c;
    USB_STR_DESCRIPTOR *pSTR;
    writeDebug('G', 'd', SPacket->wValue.H,0);
    switch (SPacket->wValue.H)
    {
    case USB_DEVICE_DESC_TYPE:
        USBLIB_SendData(0, (uint16_t *)&USB_DEVICE_DESC, sizeof(USB_DEVICE_DESC));
        break;

    case USB_CFG_DESC_TYPE:
        //USBLIB_SendData(0, (uint16_t *)&USBD_CDC_CFG_DESCRIPTOR, sizeof(USBD_CDC_CFG_DESCRIPTOR));
        break;

    case USB_STR_DESC_TYPE:
       // pSTR = (USB_STR_DESCRIPTOR *)&wLANGID;

        for (c = 0; c < SetupPacket->wValue.L; c++) {
            pSTR = (USB_STR_DESCRIPTOR *)((uint8_t *)pSTR + pSTR->bLength);
        }
        USBLIB_SendData(0, (uint16_t *)pSTR, pSTR->bLength);
        break;
    default:
        USBLIB_SendData(0, 0, 0);
        break;
    }
}

void USBLIB_SendData(uint8_t EPn, uint16_t *Data, uint16_t Length)
{
    EpData[EPn].lTX = Length;
    EpData[EPn].pTX_BUFF = Data;
    if (Length > 0)
        USBLIB_EPBuf2Pma(EPn);
      else
        BDTable[EPn].TX_Count = 0;
    USBLIB_setStatTx(EPn, TX_VALID);
}

//This routine moving data from PMA USB buffer to user buffer
void USBLIB_Pma2EPBuf(uint8_t EPn)
{
    uint32_t *Address = (uint32_t *)(USB_PMAADDR + BDTable[EPn].RX_Address * 2);
    uint16_t *Distination = (uint16_t *)EpData[EPn].pRX_BUFF;
    uint8_t   Count = EpData[EPn].lRX = (BDTable[EPn].RX_Count & BDT_COUNTn_RX_Msk); 
    
    for (uint8_t i = 0; i < Count; i++)
    {
        *(uint16_t *)Distination = *(uint16_t *)Address;
        Distination++;
        Address++;
    }
}

//This routine moving data from user buffer to PMA USB buffer
void USBLIB_EPBuf2Pma(uint8_t EPn)
{
    uint32_t *Distination;
    uint8_t   Count;

    Count  = EpData[EPn].lTX <= EpData[EPn].TX_Max ? EpData[EPn].lTX : EpData[EPn].TX_Max;
    BDTable[EPn].TX_Count = Count;
    Distination = (uint32_t *)(USB_PMAADDR + BDTable[EPn].TX_Address * 2);
    writeDebug('T', 'x', EpData[EPn].lTX,0); // will transmit xxx bytes
    for (uint8_t i = 0; i < (Count + 1) / 2; i++) 
    {
        *(uint32_t *)Distination = *(uint16_t *)EpData[EPn].pTX_BUFF;
        Distination++;
        EpData[EPn].pTX_BUFF++;
    }
    EpData[EPn].lTX -= Count; //why ???? 
}

//This routine togled EP_STAT_TX bits. Pay attention this bits for can't been directly writen, but only togle
void USBLIB_setStatTx(uint8_t EPn, uint16_t Stat)
{
    register uint16_t val = USB->EPR[EPn];
    USB->EPR[EPn] = (val ^ (Stat & EP_STAT_TX)) & (EP_MASK | EP_STAT_TX);
}

void USBLIB_setStatRx(uint8_t EPn, uint16_t Stat)
{
    register uint16_t val = USB->EPR[EPn];
    USB->EPR[EPn]         = (val ^ (Stat & EP_STAT_RX)) & (EP_MASK | EP_STAT_RX);
}

void writeDebug(char com1, char com2,uint16_t reg16, uint32_t reg32)
{
  static uint16_t limit=0;
  if (limit<100)
  {
    debug[limit].tick_=GetTick();
    debug[limit].com_1=com1;
    debug[limit].com_2=com2;
    debug[limit].reg_16= reg16;
    debug[limit].reg_32= reg32;
    limit++;
  } 
  else
    while(1)
      asm("nop");
}
