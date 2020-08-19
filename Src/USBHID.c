#include "USBHID.h"
#include "HidSensorSpec.h"
#include <stdlib.h>
#include "Systick.h"
#include "additional_func.h"
#define EPCOUNT 2

void loggingSetupPacket(USBLIB_SetupPacket *pSetup);
char debugBuf[512];

__no_init volatile USB_BDT BDTable[EPCOUNT] @ USB_PMAADDR ; //Buffer Description Table


USB_EPinfo EpData[EPCOUNT] =
{
  {.Number=0, .Type=EP_CONTROL,     .TX_Max=8, .RX_Max=8, .pTX_BUFF=0, .lTX=0, .pRX_BUFF=0, .lRX=0},
  {.Number=1, .Type=EP_INTERRUPT,   .TX_Max=8, .RX_Max=8, .pTX_BUFF=0, .lTX=0, .pRX_BUFF=0, .lRX=0}
};
USBLIB_SetupPacket   *SetupPacket;
volatile uint8_t      DeviceAddress = 0;

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
.iProduct = 2,
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
        184,0x00,           //wDescriptorLength (report descriptor lenth)
 
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

static const uint8_t HID_Sensor_ReportDesc[184] =
{
  HID_USAGE_PAGE_SENSOR,
  HID_USAGE_SENSOR_TYPE_ENVIRONMENTAL_TEMPERATURE,
  HID_COLLECTION(Physical),

  //feature reports (xmit/receive)
  HID_USAGE_PAGE_SENSOR,

  HID_USAGE_SENSOR_PROPERTY_REPORT_INTERVAL,
  HID_LOGICAL_MIN_8(0),
  HID_LOGICAL_MAX_32(0xFF,0xFF,0xFF,0xFF),
  HID_REPORT_SIZE(32),
  HID_REPORT_COUNT(1),
  HID_UNIT_EXPONENT(0),
  HID_FEATURE(Data_Var_Abs),

  HID_USAGE_SENSOR_DATA(HID_USAGE_SENSOR_DATA_ENVIRONMENTAL_TEMPERATURE,HID_USAGE_SENSOR_DATA_MOD_MAX),
  HID_LOGICAL_MIN_16(0x01,0x80), // LOGICAL_MINIMUM (-32767)
  HID_LOGICAL_MAX_16(0xFF,0x7F), // LOGICAL_MAXIMUM (32767)
  HID_REPORT_SIZE(16),
  HID_REPORT_COUNT(1),
  HID_UNIT_EXPONENT(0x0E), // scale default unit “Celsius” to provide 2 digits past the decimal point
  HID_FEATURE(Data_Var_Abs),

  HID_USAGE_SENSOR_DATA(HID_USAGE_SENSOR_DATA_ENVIRONMENTAL_TEMPERATURE,HID_USAGE_SENSOR_DATA_MOD_MIN),
  HID_LOGICAL_MIN_16(0x01,0x80), // LOGICAL_MINIMUM (-32767)
  HID_LOGICAL_MAX_16(0xFF,0x7F), // LOGICAL_MAXIMUM (32767)
  HID_REPORT_SIZE(16),
  HID_REPORT_COUNT(1),
  HID_UNIT_EXPONENT(0x0E), // scale default unit “Celsius” to provide 2 digits past the decimal point
  HID_FEATURE(Data_Var_Abs),

  //input reports (transmit)
  HID_USAGE_PAGE_SENSOR,

  HID_USAGE_SENSOR_STATE,
  HID_LOGICAL_MIN_8(0),
  HID_LOGICAL_MAX_8(6),
  HID_REPORT_SIZE(8),
  HID_REPORT_COUNT(1),
  HID_COLLECTION(Logical),
  HID_USAGE_SENSOR_STATE_UNKNOWN,
  HID_USAGE_SENSOR_STATE_READY,
  HID_USAGE_SENSOR_STATE_NOT_AVAILABLE,
  HID_USAGE_SENSOR_STATE_NO_DATA,
  HID_USAGE_SENSOR_STATE_INITIALIZING,
  HID_USAGE_SENSOR_STATE_ACCESS_DENIED,
  HID_USAGE_SENSOR_STATE_ERROR,
  HID_INPUT(Data_Arr_Abs),
  HID_END_COLLECTION,

  HID_USAGE_SENSOR_EVENT,
  HID_LOGICAL_MIN_8(0),
  HID_LOGICAL_MAX_8(16),
  HID_REPORT_SIZE(8),
  HID_REPORT_COUNT(1),
  HID_COLLECTION(Logical),
  HID_USAGE_SENSOR_EVENT_UNKNOWN,
  HID_USAGE_SENSOR_EVENT_STATE_CHANGED,
  HID_USAGE_SENSOR_EVENT_PROPERTY_CHANGED,
  HID_USAGE_SENSOR_EVENT_DATA_UPDATED,
  HID_USAGE_SENSOR_EVENT_POLL_RESPONSE,
  HID_USAGE_SENSOR_EVENT_CHANGE_SENSITIVITY,
  HID_USAGE_SENSOR_EVENT_MAX_REACHED,
  HID_USAGE_SENSOR_EVENT_MIN_REACHED,
  HID_USAGE_SENSOR_EVENT_HIGH_THRESHOLD_CROSS_UPWARD,
  HID_USAGE_SENSOR_EVENT_HIGH_THRESHOLD_CROSS_DOWNWARD,
  HID_USAGE_SENSOR_EVENT_LOW_THRESHOLD_CROSS_UPWARD,
  HID_USAGE_SENSOR_EVENT_LOW_THRESHOLD_CROSS_DOWNWARD,
  HID_USAGE_SENSOR_EVENT_ZERO_THRESHOLD_CROSS_UPWARD,
  HID_USAGE_SENSOR_EVENT_ZERO_THRESHOLD_CROSS_DOWNWARD,
  HID_USAGE_SENSOR_EVENT_PERIOD_EXCEEDED,
  HID_USAGE_SENSOR_EVENT_FREQUENCY_EXCEEDED,
  HID_USAGE_SENSOR_EVENT_COMPLEX_TRIGGER,
  HID_INPUT(Data_Arr_Abs),
  HID_END_COLLECTION,

  HID_USAGE_SENSOR_DATA_ENVIRONMENTAL_TEMPERATURE,
  HID_LOGICAL_MIN_16(0x01,0x80), // LOGICAL_MINIMUM (-32767)
  HID_LOGICAL_MAX_16(0xFF,0x7F), // LOGICAL_MAXIMUM (32767)
  HID_REPORT_SIZE(16),
  HID_REPORT_COUNT(1),
  HID_UNIT_EXPONENT(0x0E), // scale default unit “Celsius” to provide 2 digits past the decimal point
  HID_INPUT(Data_Var_Abs),
  HID_END_COLLECTION
}; 

//*****************************************************************************
//                         USB interrupt handler
void USB_LP_CAN1_RX0_IRQHandler()
{
    if (USB->ISTR & USB_ISTR_RESET) 
    {
        loggingEvent("\r\nReset. ISTR=",USB->ISTR);
        USB->ISTR &= ~USB_ISTR_RESET;
        USB_Reset();
        return;
    }
    if (USB->ISTR & USB_ISTR_CTR) 
    {
        loggingEvent("\r\nEP handling ISTR=",USB->ISTR);
        USB_EPHandler((uint16_t)USB->ISTR);
        USB->ISTR &= ~USB_ISTR_CTR;
        return;
    }
    if (USB->ISTR & USB_ISTR_PMAOVR) 
    {
        USB->ISTR &= ~USB_ISTR_PMAOVR;
        return;
    }
    if (USB->ISTR & USB_ISTR_SUSP) 
    {
        //loggingEvent("\r\nSuspend ISTR=",USB->ISTR);
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
        loggingEvent("\r\nERROR. ISTR=",USB->ISTR);
        USB->ISTR &= ~USB_ISTR_ERR;
        return;
    }
    if (USB->ISTR & USB_ISTR_WKUP) 
    {
        loggingEvent("\r\nWake Up ISTR=",USB->ISTR);
        USB->ISTR &= ~USB_ISTR_WKUP;
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
    USB->ISTR = 0;
}

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
      EpData[i].status = EP_STATUS_HALT_OFF;
      USB->EPR[i] = (EpData[i].Number | EpData[i].Type | RX_VALID | TX_NAK);
    }
    for (uint8_t i = EPCOUNT; i < 8; i++) //inactive endpoints
    {
      USB->EPR[i] = i | RX_NAK | TX_NAK;
      EpData[i].status = EP_STATUS_HALT_ON;
    }
    
    USB->CNTR   = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_ERRM;
    USB->ISTR   = 0x00;
    USB->BTABLE = 0x00;
    USB->DADDR  = USB_DADDR_EF; //Enable USB device
}

//*****************************************************************************
void USB_EPHandler(uint16_t Status)
{
    uint16_t DeviceConfigured = 0;
    uint16_t DeviceStatus = 0;      //BUS_POWERED, NOT Remote WakeUp (Figure 9-4 USB 2.0 specification)
    uint8_t  EPn = Status & ISTR_EP_ID; //endpoint number where occured
    uint32_t EP  = USB->EPR[EPn];
    if (EP & EP_CTR_RX)     //something received ?
    { 
        loggingEvent("\r\nReceived from EPn=",EPn);
        USBLIB_Pma2EPBuf(EPn);
        if (EPn == 0)       //Control endpoint ?
        { 
          if (EP & USB_EP0R_SETUP) //Setup packet ?
          {
            SetupPacket = (USBLIB_SetupPacket *)EpData[EPn].pRX_BUFF;
            loggingSetupPacket(SetupPacket);
            if ((SetupPacket->bmRequestType & USB_REQUEST_TYPE) == USB_REQUEST_STANDARD)	//Request type Standard ?
            {
              debugprint("\r\nStandard Request Type ");
              switch (SetupPacket->bRequest) 
              {
                case USB_REQUEST_GET_DESCRIPTOR:
                    debugprint("Get Descriptor");
                    USBLIB_GetDescriptor(SetupPacket);
                    break;
                    
                case USB_REQUEST_SET_ADDRESS:
                    debugprint("Set Address");
                    USBLIB_SendData(0, 0, 0);
                    DeviceAddress = SetupPacket->wValue.L;
                    break;

                case USB_REQUEST_GET_STATUS:
                    debugprint("Get Status");
                    if ((SetupPacket->bmRequestType & USB_REQUEST_RECIPIENT) == USB_REQUEST_DEVICE)	//Recipient is Device ?
                      USBLIB_SendData(0, &DeviceStatus, 2);
                    if ((SetupPacket->bmRequestType & USB_REQUEST_RECIPIENT) == USB_REQUEST_ENDPOINT)	//Recipient is Endpoint ?
                      USBLIB_SendData(0, &EpData[SetupPacket->wIndex.L].status, 2);
                    break;

                case USB_REQUEST_GET_CONFIGURATION:
                    debugprint("Get Config");
                    USBLIB_SendData(0, &DeviceConfigured, 1);
                    break;

                case USB_REQUEST_SET_CONFIGURATION:
                    debugprint("Set Conf");
                    DeviceConfigured = 1;
                    USBLIB_SendData(0, 0, 0);
                    break;
                    
                case USB_REQUEST_GET_INTERFACE:
                  debugprint("Get Interface");
                  USBLIB_SendData(0, 0, 0);
                  break;

                case USB_REQUEST_CLEAR_FEATURE:
                  debugprint("Clear Feature");
                  USBLIB_SendData(0, 0, 0);
                  if ((SetupPacket->bmRequestType & USB_REQUEST_RECIPIENT) == USB_REQUEST_DEVICE)	//Recipient is Device ?
                    if(SetupPacket->wValue.L == 1)
                      DeviceStatus &= (uint16_t)~STATUS_REMOTE_WAKEUP;
                  
                  if ((SetupPacket->bmRequestType & USB_REQUEST_RECIPIENT) == USB_REQUEST_ENDPOINT) //Recipient is Endpoint ?    
                    {
                      EpData[SetupPacket->wIndex.L].status = SetupPacket->wValue.L;
                      
                    }
                  break;
                  
                default:
                  debugprint("??");
              }//switch (SetupPacket->bRequest)
            }//Request type Standard
            
            else if ((SetupPacket->bmRequestType & USB_REQUEST_TYPE) == USB_REQUEST_CLASS)	//Request type Class ?
            {
              debugprint("\r\nClass Request Type ");
              switch (SetupPacket->bRequest) 
              {
                case USB_HID_GET_REPORT:
                 debugprint("Get Report");
                  break;
                case USB_HID_SET_REPORT:
                  debugprint("Set Report");
                  break;
                case USB_HID_GET_IDLE:
                  debugprint("Get Idle");
                  break;  
                case USB_HID_SET_IDLE:
                  debugprint("Set Idle");
                  break;
                case USB_HID_GET_PROTOCOL:
                  debugprint("Get Protocol");
                  break;
                case USB_HID_SET_PROTOCOL:
                  debugprint("Set protocol");
                  break;
                default:
                  debugprint("??");
                  break;
              }//switch (SetupPacket->bRequest) 
            }//Request type Class ?
          }//Setup packet
          else
            loggingEvent("\r\nNot Setup Packet EP[0]=",(uint16_t)USB->EPR[0]);
        }//Control endpoint 
        
        else 
        { // Got data from another EP
         loggingEvent("\r\nGot from  EP=",EPn);
        //   uUSBLIB_DataReceivedHandler(EpData[EPn].pRX_BUFF, EpData[EPn].lRX);
        }
        USB->EPR[EPn] &= 0x78f; //reset flag CTR_RX
        USBLIB_setStatRx(EPn, RX_VALID);
    }//something received ?
    
    if (EP & EP_CTR_TX) //something transmitted
      { 
        debugprint("\r\nTransmit Data ");
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
            debugprint("\r\nEnd Transmit ");
            //uUSBLIB_DataTransmitedHandler(EPn, EpData[EPn]);
          }
        USB->EPR[EPn] &= 0x870f;   //reset flag CTR_TX
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
        descSize=(sizeof(sDeviceDescriptor)< SPacket->wLength)? sizeof(sDeviceDescriptor) : SPacket->wLength;
        USBLIB_SendData(0, (uint16_t *)&sDeviceDescriptor, descSize);
        break;

    case USB_CFG_DESC_TYPE:
      //use only one configuration, but wValue.L is ignored
        descSize=(sizeof(aConfDescriptor)< SPacket->wLength)? sizeof(aConfDescriptor) : SPacket->wLength;
        USBLIB_SendData(0, (uint16_t *)&aConfDescriptor, descSize);
        break;

    case USB_STR_DESC_TYPE:
        pSTR = (USB_STR_DESCRIPTOR *)StringDescriptors[SetupPacket->wValue.L];
        descSize=(pSTR->bLength < SPacket->wLength)? pSTR->bLength : SPacket->wLength;
        USBLIB_SendData(0, (uint16_t *)pSTR, descSize);
        break;
        
    case USB_DEVICE_QR_DESC_TYPE:
        /*descSize=(sizeof(sQualDescriptor)< SPacket->wLength)? sizeof(sQualDescriptor) : SPacket->wLength;
        USBLIB_SendData(0, (uint16_t *)&sQualDescriptor, descSize);
        break;*/
        BDTable[0].TX_Count = 0;
        USBLIB_setStatTx(0, TX_STALL);
        break;
       
    case USB_REPORT_DESC_TYPE:
        descSize=(sizeof(HID_Sensor_ReportDesc)< SPacket->wLength)? sizeof(HID_Sensor_ReportDesc) : SPacket->wLength;
        USBLIB_SendData(0, (uint16_t *)&HID_Sensor_ReportDesc, descSize);
        break;    
        
    default:
        USBLIB_SendData(0, 0, 0);
        break;
    }
}

//*****************************************************************************
void USBLIB_SendData(uint8_t EPn, uint16_t *Data, uint16_t Length)
{
    loggingEvent("\r\nSend len=",Length);
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
    loggingEvent("\r\nReceive bytes=",Count);
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
    //writeDebug('T', 'x', EpData[EPn].lTX,(uint32_t)EpData[EPn].pTX_BUFF); // will transmit xxx bytes
    
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
    USB->EPR[EPn] = (val ^ (Stat & EP_STAT_RX)) & (EP_MASK | EP_STAT_RX);
}

/* ****************************************************************************
SWO logging for SETUP packet type to SWO by ITM_SendChar
pSetup - pointer on struct
**************************************************************************** */
void loggingSetupPacket(USBLIB_SetupPacket *pSetup)
{
  char *pFloat = stradd (debugBuf, "\r\n-------------\r\nTime=");
  pFloat = itoa(GetTick() ,pFloat,10,0);
  pFloat = stradd (pFloat, "\r\nbmRequestType=");
  pFloat = itoa(pSetup->bmRequestType ,pFloat,2,0);
  pFloat = stradd (pFloat, "\r\nbRequest=");
  pFloat = itoa(pSetup->bRequest ,pFloat,10,0);
  pFloat = stradd (pFloat, "\r\nwValue.H= ");
  pFloat = itoa(pSetup->wValue.H ,pFloat,10,0);
  pFloat = stradd (pFloat, "\r\nwValue.L= ");
  pFloat = itoa(pSetup->wValue.L ,pFloat,10,0);
  pFloat = stradd (pFloat, "\r\nwIndex= ");
  pFloat = itoa(pSetup->wIndex.L,pFloat,2,0);
  pFloat = stradd (pFloat, "\r\nwLength= ");
  pFloat = itoa(pSetup->wLength ,pFloat,10,0);
  debugprint (debugBuf);
  return;
}

void loggingEvent(char *eventdesc, uint32_t reg)
{
  char *pFloat = stradd (debugBuf, "\r\n-------------\r\nTime=");
  pFloat = itoa(GetTick() ,pFloat,10,0);
  pFloat = stradd (pFloat, (char*)eventdesc);
  pFloat = itoa(reg ,pFloat,2,0);
  pFloat = stradd (pFloat, " (0x");
  pFloat = itoa(reg ,pFloat,16,0);
  pFloat = stradd (pFloat, ")\0");
  debugprint (debugBuf);
} 