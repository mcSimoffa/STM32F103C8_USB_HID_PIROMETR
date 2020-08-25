#include "USBHID.h"
#include "HidSensorSpec.h"
#include <stdlib.h>
#include "Systick.h"
#include "additional_func.h"
#include "oringbuf.h"

#define DEVICE_VENDOR_ID        0x25AE
#define DEVICE_PRODUCT_ID       0x24AB
#define EPCOUNT                 2
#define LAST_NUM_STRING_DESCR   2

__no_init volatile USB_BDT BDTable[EPCOUNT] @ USB_PMAADDR ; //Buffer Description Table

#ifdef SWOLOG
  void loggingSetupPacket(USBLIB_SetupPacket *pSetup);
  void putlog();
  char debugBuf[512];
  char *pFloat;
#endif
  
USB_EPinfo EpData[EPCOUNT] =
{
  {.Number=0, .Type=EP_CONTROL,     .TX_Max=8, .RX_Max=8, .pTX_BUFF=0, .lTX=0, .pRX_BUFF=0, .lRX=0},
  {.Number=1, .Type=EP_INTERRUPT,   .TX_Max=8, .RX_Max=8, .pTX_BUFF=0, .lTX=0, .pRX_BUFF=0, .lRX=0}
};
USBLIB_SetupPacket   *SetupPacket;
volatile uint8_t      DeviceAddress = 0;
uint8_t reportPeriod=0;

/*const Typedef_USB_DEVICE_QUALIFIER_DESCRIPTOR sQualDescriptor={
.bLength = 10, 
.bDescriptorType = USB_DEVICE_QR_DESC_TYPE,
.bcdUSB_L = 0,
.bcdUSB_H = 2,
.bDeviceClass = USB_CLASS_IN_INTERFACE_DESCRIPTOR,
.bDeviceSubClass = 0,
.bDeviceProtocol = 0,
.bMaxPacketSize0 = 8,
.bNumConfigurations =0,
.bReservedFuture=0};*/

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
.iProduct = LAST_NUM_STRING_DESCR,
.iSerialNumber = 0,
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
        0x11,  0x01,        //bBCDHID low & high (ver 1.12)   
        0x00,               //bCountryCode (not Localisation)
        0x01,               //bNumDescriptors (follow 1 report descriptor)
        USB_REPORT_DESC_TYPE,   //bDescriptorType (report)
        23,0x00,           //wDescriptorLength (report descriptor lenth)
 
// ENDPOINT descriptor   (Table 9-13 USB specification) 
        0x07,                       //bLength:
        USB_EP_DESC_TYPE,           //bDescriptorType
        IN_ENDPOINT | 0x01,         //bEndpointAddress(ep#1 direction=IN)
        EP_TRANSFER_TYPE_INTERRUPT, // bmAttributes
        0x08, 0x00,                 // wMaxPacketSize: 8 Byte max 
        0x20                        // bInterval: Polling Interval (32 ms)           
    };

const uint8_t aStringDescriptors0[]=
{
    0x04,               // bLength
    (uint8_t)USB_STR_DESC_TYPE,  //bDescriptorType
    0x09,0x04               //LANG_ID English
};
const uint8_t aStringDescriptors1[]=
{
    36,                 // bLength
    USB_STR_DESC_TYPE,  //bDescriptorType
    'M',0, 'a',0, 'x',0, 'i',0, 'm',0, 'o',0, ' ',0, 'T',0, 'e',0, 'c',0, 'h',0, 'n',0, 'o',0, 'l',0, 'o',0, 'g',0, 'y',0   //Manufactured
};

const uint8_t aStringDescriptors2[]=
{
    20,                 // bLength
    USB_STR_DESC_TYPE,  //bDescriptorType
    'P',0, 'i',0, 'r',0, 'o',0, 'm',0, 'e',0, 't',0, 'e',0, 'r',0, //Product
};

uint8_t* StringDescriptors[3]=
{
  (uint8_t*) &aStringDescriptors0,
  (uint8_t*) &aStringDescriptors1,
  (uint8_t*) &aStringDescriptors2
};

static const uint8_t HID_Sensor_ReportDesc[23] =
{

  0x06, 0x00, 0xff,		// USAGE_PAGE (Generic Desktop)
  0x09, 0x01,			// USAGE (Vendor Usage 1)
  0xa1, 0x01,			// COLLECTION (Application)
  0x19, 0x01,			// USAGE_MINIMUM (Vendor Usage 1)
  0x29, 0x01,			// USAGE_MAXIMUM (Vendor Usage 1)
  0x15, 0x00,			// LOGICAL_MINIMUM (0)
  0x26, 0xff, 0x00,		// LOGICAL_MAXIMUM (255)
  0x75, 0x08,			// REPORT_SIZE (8)
  0x95, 64,				// REPORT_COUNT(64)
  0xB1, 0x02,			//FEATURE (Data,Var,Abs)
  0xc0					//END_COLLECTION
}; 

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
        {
          EpData[i].pRX_BUFF = (uint16_t *)malloc(EpData[i].RX_Max);
        #ifdef DEBUG
          if(EpData[i].pRX_BUFF == 0)
            while(1)
              asm("nop");         //memory not provided
        #endif   
        }
      USB->EPR[i] = (EpData[i].Number | EpData[i].Type | RX_VALID | TX_NAK);
      }

    for (uint8_t i = EPCOUNT; i < 8; i++) //inactive endpoints
        USB->EPR[i] = i | RX_NAK | TX_NAK;

    USB->CNTR   = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_ERRM;
    USB->ISTR   = 0x00;
    USB->BTABLE = 0x00;
    USB->DADDR  = USB_DADDR_EF; //Enable USB device
}

//*****************************************************************************
void USB_LP_CAN1_RX0_IRQHandler()
{
    DWT->CYCCNT = 0;
    uint16_t istr=(uint16_t)USB->ISTR;
  #ifdef SWOLOG
    pFloat = stradd (debugBuf, " ");
  #endif
      
    if (USB->ISTR & USB_ISTR_SUSP) 
    {
      USB->ISTR &= ~USB_ISTR_SUSP;
    #ifdef SWOLOG
      pFloat = stradd (pFloat, "S");
      putlog();
    #endif
      if (USB->DADDR & 0x7f) 
      {
        USB->DADDR = 0;
        USB->CNTR &= ~ USB_CNTR_SUSPM;
        USB->CNTR |= USB_CNTR_WKUPM;
      #ifdef SWOLOG  
        pFloat = stradd (pFloat, "\r\nISTR=");
        pFloat = itoa(istr, pFloat,2,0);
        pFloat = stradd (pFloat, "\r\nFNR=");
        pFloat = itoa(USB->FNR ,pFloat,2,0);
        pFloat = stradd (pFloat, "\r\nEP0=");
        pFloat = itoa(USB->EPR[0] ,pFloat,2,0);
        putlog();
      #endif  
      }
      return;
    }
  #ifdef SWOLOG
    pFloat = stradd (pFloat, "\r\n\n---------\r\nTime=");
    pFloat = itoa(GetTick(), pFloat,10,0);
    pFloat = stradd (pFloat, "\r\nISTR=");
    pFloat = itoa(istr, pFloat,2,0);
    pFloat = stradd (pFloat, "\r\nFNR=");
    pFloat = itoa(USB->FNR ,pFloat,2,0);
  #endif
      
    if (USB->ISTR & USB_ISTR_RESET) 
    { // Reset
      USB->ISTR &= ~USB_ISTR_RESET;
    #ifdef SWOLOG
      pFloat = stradd (pFloat, "\r\nRESET");
      putlog();
    #endif      
      USB_Reset();
      return;
    }
    if (USB->ISTR & USB_ISTR_CTR) 
    { //Handle data on EP
      USB->ISTR &= ~USB_ISTR_CTR;
      USB_EPHandler((uint16_t)USB->ISTR);
      putlog();
      return;
    }
    if (USB->ISTR & USB_ISTR_PMAOVR) 
    {
      USB->ISTR &= ~USB_ISTR_PMAOVR;
      // Handle PMAOVR status
      return;
    }
    
    if (USB->ISTR & USB_ISTR_ERR) 
    {
      USB->ISTR &= ~USB_ISTR_ERR;
  #ifdef SWOLOG
      pFloat = stradd (pFloat, "\r\nERR");
      putlog();
  #endif
      return;
    }
    
    if (USB->ISTR & USB_ISTR_WKUP) 
    {      
      USB->ISTR &= ~USB_ISTR_WKUP;
      USB->CNTR &= ~ USB_CNTR_WKUPM;
      USB->CNTR |= USB_CNTR_SUSPM;
    #ifdef SWOLOG
      pFloat = stradd (pFloat, "\r\nWKUP");
      putlog();       
    #endif
      return;
    }
    
    if (USB->ISTR & USB_ISTR_SOF) 
    {
      USB->ISTR &= ~USB_ISTR_SOF;
      return;
    }
    
    if (USB->ISTR & USB_ISTR_ESOF) 
    {
      USB->ISTR &= ~USB_ISTR_ESOF;
      return;
    }
    USB->ISTR = 0;                                                               //kogda eto ispolnaetsa
}

//*****************************************************************************
void USB_EPHandler(uint16_t Status)
{
    uint16_t DeviceConfigured = 0, DeviceStatus = 0;
    uint8_t  EPn = Status & ISTR_EP_ID; //endpoint number where occured
    uint32_t EP  = USB->EPR[EPn];
  #ifdef SWOLOG
    pFloat = stradd (pFloat,"\r\nEP=");
    pFloat = itoa(EP ,pFloat,2,0);
  #endif
    if (EP & EP_CTR_RX)     //something received ?
    {
      USB->EPR[EPn] &= 0x78f; //reset flag CTR_RX
    #ifdef SWOLOG
      pFloat = stradd (pFloat,"\r\n received on EPnum=");
      pFloat = itoa(EPn ,pFloat,10,0);
    #endif
      USBLIB_Pma2EPBuf(EPn);
      if (EPn == 0)       //Control endpoint ?
      { 
        if (EP & USB_EP0R_SETUP) 
        {
        SetupPacket = (USBLIB_SetupPacket *)EpData[EPn].pRX_BUFF;
      #ifdef SWOLOG
        pFloat = stradd (pFloat,"\r\nSETUP ");
        loggingSetupPacket(SetupPacket);
      #endif
        if ((SetupPacket->bmRequestType & USB_REQUEST_TYPE) == USB_REQUEST_STANDARD)	//Request type Standard ?
        {
      #ifdef SWOLOG
          pFloat = stradd (pFloat,"\r\nSTANDARD ");
      #endif
          switch (SetupPacket->bRequest) 
          {
          case USB_REQUEST_GET_DESCRIPTOR:
          #ifdef SWOLOG
            pFloat = stradd (pFloat,"GET_DESCRIPTOR ");
          #endif
            USBLIB_GetDescriptor(SetupPacket);
            break;
              
          case USB_REQUEST_SET_ADDRESS:
          #ifdef SWOLOG
            pFloat = stradd (pFloat,"SET_ADDRESS");
          #endif
            USBLIB_SendData(0, 0, 0);
            DeviceAddress = SetupPacket->wValue.L;
            break;

          case USB_REQUEST_GET_STATUS:
          #ifdef SWOLOG
            pFloat = stradd (pFloat,"GET_STATUS");
          #endif
            USBLIB_SendData(0, &DeviceStatus, 2);     // I don't sure.... !!!!!!!!!!!!!!!!!!
            break;

          case USB_REQUEST_GET_CONFIGURATION:
          #ifdef SWOLOG
            pFloat = stradd (pFloat,"GET_CONF");
          #endif
            USBLIB_SendData(0, &DeviceConfigured, 1);
            break;

          case USB_REQUEST_SET_CONFIGURATION:
          #ifdef SWOLOG
            pFloat = stradd (pFloat,"SET_CONF");
          #endif
            //if wValue.L= 0  State = Adresovano
            //else need STALL answer
            if (SetupPacket->wValue.L == 1)
              DeviceConfigured = 1;   //if only one configuration - allowed so
            USBLIB_SendData(0, 0, 0);
            break;
              
          case USB_REQUEST_GET_INTERFACE:
          #ifdef SWOLOG
             pFloat = stradd (pFloat,"GET_INTFACE");
          #endif
            USBLIB_SendData(0, 0, 0);        // I don't sure....
            break;
       
          case USB_REQUEST_CLEAR_FEATURE:
          #ifdef SWOLOG
             pFloat = stradd (pFloat,"CLEAR_FEATURE");
          #endif
            USBLIB_SendData(0, 0, 0);        // I don't sure....
            break;
            
          default:
            asm("nop");
          #ifdef SWOLOG
            pFloat = stradd (pFloat,"??");
          #endif
          }//switch (SetupPacket->bRequest)
        }//Request type Standard
        
        else if ((SetupPacket->bmRequestType & USB_REQUEST_TYPE) == USB_REQUEST_CLASS)	
        { //Request type Class ?
        #ifdef SWOLOG
          pFloat = stradd (pFloat," CLASS ");
        #endif
          switch (SetupPacket->bRequest) 
          {
            case USB_HID_GET_REPORT:
            #ifdef SWOLOG
              pFloat = stradd (pFloat,"Get Report");
            #endif
            break;
                                  
            case USB_HID_SET_REPORT:
            #ifdef SWOLOG
              pFloat = stradd (pFloat,"Set Report");
            #endif  
            break;
                                  
            case USB_HID_GET_IDLE:
            #ifdef SWOLOG
              pFloat = stradd (pFloat,"Get Idle");
            #endif          
              break; 
                                  
            case USB_HID_SET_IDLE:
            #ifdef SWOLOG
              pFloat = stradd (pFloat,"Set Idle");
            #endif
            reportPeriod = SetupPacket->wValue.H;
            //USBLIB_SendData(0, 0, 0);                                          skoree vsego na SWT IDLE ne nado otvechat
            break;
                                  
            case USB_HID_GET_PROTOCOL:
            #ifdef SWOLOG
              pFloat = stradd (pFloat,"Get Protocol");
            #endif  
            break;
                                  
            case USB_HID_SET_PROTOCOL:
            #ifdef SWOLOG
              pFloat = stradd (pFloat,"Set protocol");
            #endif  
            break;
                                  
            default:
            #ifdef SWOLOG
              pFloat = stradd (pFloat,"??");
            #endif  
            break;
          }//switch (SetupPacket->bRequest) 
        }//Request type Class ?
      }//Setup packet
    }//Control endpoint 
    else 
      { // Got data from another EP
        asm("nop");  // Call user function
      #ifdef SWOLOG
        pFloat = stradd (pFloat," OUT__");
      #endif         
       //   uUSBLIB_DataReceivedHandler(EpData[EPn].pRX_BUFF, EpData[EPn].lRX);
      }
      
      USBLIB_setStatRx(EPn, RX_VALID);
    }//something received
    
    if (EP & EP_CTR_TX) //something transmitted
      {
        USB->EPR[EPn] &= 0x870f;   //reset flag CTR_TX
      #ifdef SWOLOG
        pFloat = stradd (pFloat, "\r\nTransmit by EPnum=");
        pFloat = itoa(EPn ,pFloat,10,0);
      #endif       
        if (DeviceAddress) 
          {
            USB->DADDR    = DeviceAddress | 0x80;
            DeviceAddress = 0;
          }
        if (EpData[EPn].lTX) //Have to transmit something?
          {           
            USBLIB_EPBuf2Pma(EPn);
            USBLIB_setStatTx(EPn, TX_VALID);
          } 
        else 
          {
          #ifdef SWOLOG
            pFloat = stradd (pFloat," End Transmit ");
          #endif
          //DataTransmitedHandler();
          }
      }//something transmitted
}//void USB_EPHandler

//*****************************************************************************
//see chapter 9.4.3 USB 2.0 specification
void USBLIB_GetDescriptor(USBLIB_SetupPacket *SPacket)
{
  USB_STR_DESCRIPTOR *pSTR;
  uint16_t descSize;
  switch (SPacket->wValue.H)
  {
    case USB_DEVICE_DESC_TYPE:
    #ifdef SWOLOG
      pFloat = stradd (pFloat,"DEVICE");
    #endif
      descSize=(sizeof(sDeviceDescriptor)< SPacket->wLength)? sizeof(sDeviceDescriptor) : SPacket->wLength;
      USBLIB_SendData(0, (uint16_t *)&sDeviceDescriptor, descSize);
      break;

    case USB_CFG_DESC_TYPE:
    #ifdef SWOLOG
      pFloat = stradd (pFloat,"CONF");
    #endif  
    //use only one configuration, but wValue.L is ignored
      descSize=(sizeof(aConfDescriptor)< SPacket->wLength)? sizeof(aConfDescriptor) : SPacket->wLength;
      USBLIB_SendData(0, (uint16_t *)&aConfDescriptor, descSize);
      break;

    case USB_STR_DESC_TYPE:
    #ifdef SWOLOG
      pFloat = stradd (pFloat,"STR");
    #endif
      if (SPacket->wValue.L > LAST_NUM_STRING_DESCR)
        {
          //USBLIB_SendData(0, 0, 0);
          BDTable[0].TX_Count = 0;
          USBLIB_setStatTx(0, TX_STALL); 
        }
        else
        {
          pSTR = (USB_STR_DESCRIPTOR *)StringDescriptors[SetupPacket->wValue.L];
          descSize=(pSTR->bLength < SPacket->wLength)? pSTR->bLength : SPacket->wLength;
          USBLIB_SendData(0, (uint16_t *)pSTR, descSize);
        }
      break;
        
    case USB_DEVICE_QR_DESC_TYPE:
    #ifdef SWOLOG
      pFloat = stradd (pFloat,"QUAL");
    #endif  
      /*descSize=(sizeof(sQualDescriptor)< SPacket->wLength)? sizeof(sQualDescriptor) : SPacket->wLength;
      USBLIB_SendData(0, (uint16_t *)&sQualDescriptor, descSize);
      break;  // if have QUALIFIER Descriptor*/
      BDTable[0].TX_Count = 0;
      USBLIB_setStatTx(0, TX_STALL);                                              //maybe EP_KIND must be 1 ?
      break;
       
    case USB_REPORT_DESC_TYPE:
    #ifdef SWOLOG
      pFloat = stradd (pFloat,"REPORT");
    #endif  
      descSize=(sizeof(HID_Sensor_ReportDesc)< SPacket->wLength)? sizeof(HID_Sensor_ReportDesc) : SPacket->wLength;
      USBLIB_SendData(0, (uint16_t *)&HID_Sensor_ReportDesc, descSize);
      break;    
        
    default:
    #ifdef SWOLOG
      pFloat = stradd (pFloat,"??");
    #endif   
      USBLIB_SendData(0, 0, 0);
      break;
  }
}

//*****************************************************************************
void USBLIB_SendData(uint8_t EPn, uint16_t *Data, uint16_t Length)
{
#ifdef SWOLOG
  pFloat = stradd (pFloat, "\r\nPrepare to transmit ");
  pFloat = itoa(Length ,pFloat,10,0);
#endif
  EpData[EPn].lTX = Length;
  EpData[EPn].pTX_BUFF = Data;
  if (Length > 0)
      USBLIB_EPBuf2Pma(EPn);
    else
      BDTable[EPn].TX_Count = 0;
                                                                                          //vozmojno zdes nado obnulit flag TC
  USBLIB_setStatTx(EPn, TX_VALID);
}

//*****************************************************************************
//This routine moving data from PMA USB buffer to user buffer
void USBLIB_Pma2EPBuf(uint8_t EPn)
{
  uint32_t *Address = (uint32_t *)(USB_PMAADDR + BDTable[EPn].RX_Address * 2);
  uint16_t *Distination = (uint16_t *)EpData[EPn].pRX_BUFF;
  uint8_t   Count = EpData[EPn].lRX = (BDTable[EPn].RX_Count & BDT_COUNTn_RX_Msk);
#ifdef SWOLOG    
  pFloat = stradd (pFloat, "\r\nPMA -> Buffer ");
  pFloat = itoa(Count ,pFloat,10,0);
#endif  
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
#ifdef SWOLOG
  pFloat = stradd (pFloat, "\r\nBuffer -> PMA ");
  pFloat = itoa(Count ,pFloat,10,0);
#endif
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

#ifdef SWOLOG
/* ****************************************************************************
SWO logging for SETUP packet type to SWO by ITM_SendChar
pSetup - pointer on struct
**************************************************************************** */
void loggingSetupPacket(USBLIB_SetupPacket *pSetup)
{
  pFloat = stradd (pFloat, "\r\nbmRequestType=");
  pFloat = itoa(pSetup->bmRequestType ,pFloat,2,0);
  pFloat = stradd (pFloat, "\r\nbRequest=");
  pFloat = itoa(pSetup->bRequest ,pFloat,10,0);
  pFloat = stradd (pFloat, "\r\nwValue.H= ");
  pFloat = itoa(pSetup->wValue.H ,pFloat,10,0);
  pFloat = stradd (pFloat, "\r\nwValue.L= ");
  pFloat = itoa(pSetup->wValue.L ,pFloat,10,0);
  pFloat = stradd (pFloat, "\r\nwIndex= ");
  pFloat = itoa(pSetup->wIndex.L,pFloat,10,0);
  pFloat = stradd (pFloat, "\r\nwLength= ");
  pFloat = itoa(pSetup->wLength ,pFloat,10,0);
  return;
}

void putlog()
{
  uint16_t len=pFloat-debugBuf;
  if (len)
    if (Oringbuf_Put(debugBuf, len)<len)
      while(1)
        asm("nop");   //debug buffer is owerflowed
  return;
}
#endif
