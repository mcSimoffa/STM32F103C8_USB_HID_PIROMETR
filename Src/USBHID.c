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

const Typedef_USB_DEVICE_DESCRIPTOR sDeviceDescriptor={
.bLength = 18, 
.bDescriptorType = USB_DEVICE_DESC_TYPE,
.bcdUSB_L = 0,
.bcdUSB_H = 2,
.bDeviceClass = USB_CLASS_IN_INTERFACE_DESCRIPTOR,
.bDeviceSubClass = 0,
.bDeviceProtocol = 0,
.bMaxPacketSize0 = 8,
.idVendor_L = LOBYTE(0x0483),
.idVendor_H = HIBYTE(0x0483),
.idProduct_L = LOBYTE(0x5711),
.idProduct_H = HIBYTE(0x5711),
.bcdDevice_L = 10,
.bcdDevice_H = 1,
.iManufacturer = 1,
.iProduct = 2,
.iSerialNumber = 3,
.bNumConfigurations =1};

const uint8_t aConfDescriptor[] = 
    {
// CONFIGURATION Descriptor  (Table 9-10 USB specification)
        0x09,               //bLength: Configuration Descriptor size
        USB_CFG_DESC_TYPE,  // bDescriptorType: Configuration
        34, 0,              // wTotalLength low & high  sizeof (configuration + interface + endpoint + HID) 34bytes
        0x01,               // bNumInterfaces: 1 interface
        0x01,               // bConfigurationValue: Configuration value
        0x00,               // iConfiguration: Index of string descriptor describing the configuration
        BMATTRIBUTES_MASK,  // bmAttributes - Bus powered
        0x32,               // MaxPower 100 mA
        
// INTERFACE descriptor (Table 9-12 USB specification)
        0x09,               //bLength
        USB_IFACE_DESC_TYPE,//bDescriptorType: Interface
        0x00,               //bInterfaceNumber
        0x00,               //bAlternateSetting
        0x01,               //bNumEndpoints
        USB_CLASS_HID,      //bInterfaceClass
        0x00,               //bInterfaceSubClass (0=not bootable or 1=bootable)
        0x00,               //bInterfaceProtocol (if not bootable than always=0)
        0x00,               //iInterface (no string Interface descriptor)
        
// HID descriptor (6.2.1 Device Class Definition for Human Interface Devices (HID) Version 1.11 )
        0x09,               //bLength
        USB_HID_DESC_TYPE,  //bDescriptorType: HID
        0x0C,  0x01,        //bBCDHID low & high (ver 1.12)   
        0x00,               //bCountryCode (not Localisation)
        0x01,               //bNumDescriptors (follow 1 report descriptor)
        USB_REPORT_DESC_TYPE,   //bDescriptorType (report)
        0x3F,0x00,           //wDescriptorLength (report descriptor lenth)
 
// ENDPOINT descriptor   (Table 9-13 USB specification) 
        0x07,                       //bLength:
        USB_EP_DESC_TYPE,           //bDescriptorType
        IN_ENDPOINT | 0x01,         //bEndpointAddress(ep#1 direction=IN)
        EP_TRANSFER_TYPE_INTERRUPT, // bmAttributes
        0x01, 0x00,                 // wMaxPacketSize: 1 Byte max 
        0x20                       // bInterval: Polling Interval (32 ms)           
    };

const struct
{
        uint8_t  bLength;
        uint8_t  bDescriptorType;
        wchar_t bString[(sizeof(L"Maximo technology"))];
}wsVendor={1,2,L"Maximo technology"};


//******************************************************************************
void USB_Reset(void)
{
    uint16_t Addr = sizeof(BDTable)>>1;
    for (uint8_t i = 0; i < EPCOUNT; i++) //active endpoints
	{
        BDTable[i].TX_Address = Addr;
        BDTable[i].TX_Count   =  0;
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

//*****************************************************************************
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

//*****************************************************************************
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

//*****************************************************************************
void USBLIB_GetDescriptor(USBLIB_SetupPacket *SPacket)
{
    uint8_t c;
    USB_STR_DESCRIPTOR *pSTR;
    writeDebug('G', 'd', SPacket->wValue.H,0);
    switch (SPacket->wValue.H)
    {
    case USB_DEVICE_DESC_TYPE:
        USBLIB_SendData(0, (uint16_t *)&sDeviceDescriptor, sizeof(sDeviceDescriptor));
        break;

    case USB_CFG_DESC_TYPE:
        USBLIB_SendData(0, (uint16_t *)&aConfDescriptor, sizeof(aConfDescriptor));
        break;

    case USB_STR_DESC_TYPE:
        //pSTR = (USB_STR_DESCRIPTOR *)&wLANGID;

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

//*****************************************************************************
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

//*****************************************************************************
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

//*****************************************************************************
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
    EpData[EPn].lTX -= Count;
}

/* ****************************************************************************
This routine togled EP_STAT_TX bits. 
Pay attention this bits for can't been directly writen, but only togle */
void USBLIB_setStatTx(uint8_t EPn, uint16_t Stat)
{
    register uint16_t val = USB->EPR[EPn];
    USB->EPR[EPn] = (val ^ (Stat & EP_STAT_TX)) & (EP_MASK | EP_STAT_TX);
}

//*****************************************************************************
void USBLIB_setStatRx(uint8_t EPn, uint16_t Stat)
{
    register uint16_t val = USB->EPR[EPn];
    USB->EPR[EPn]         = (val ^ (Stat & EP_STAT_RX)) & (EP_MASK | EP_STAT_RX);
}

//*****************************************************************************
//Debugging real-time
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
