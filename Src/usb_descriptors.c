#include "usb_descriptor.h"
#include "config.h"

#define DEVICE_VENDOR_ID        0x1209
#define DEVICE_PRODUCT_ID       0x0001
#define LAST_NUM_STRING_DESCR   3

#ifdef QUALIFIER_DESCRIPTOR
const Typedef_USB_DEVICE_QUALIFIER_DESCRIPTOR sQualDescriptor={
.bLength = 10, 
.bDescriptorType = USB_DEVICE_QR_DESC_TYPE,
.bcdUSB_L = 0,
.bcdUSB_H = 2,
.bDeviceClass = USB_CLASS_IN_INTERFACE_DESCRIPTOR,
.bDeviceSubClass = 0,
.bDeviceProtocol = 0,
.bMaxPacketSize0 = 8,
.bNumConfigurations =0,
.bReservedFuture=0};
#endif

const Typedef_USB_DEVICE_DESCRIPTOR sDeviceDescriptor={
.bLength = 18, 
.bDescriptorType = USB_DEVICE_DESC_TYPE,
.bcdUSB_L = 0,
.bcdUSB_H = 2,
.bDeviceClass = USB_CLASS_IN_INTERFACE_DESCRIPTOR,
.bDeviceSubClass = 0,
.bDeviceProtocol = 0,
.bMaxPacketSize0 = 16,
.idVendor_L = LOBYTE(DEVICE_VENDOR_ID),
.idVendor_H = HIBYTE(DEVICE_VENDOR_ID),
.idProduct_L = LOBYTE(DEVICE_PRODUCT_ID),
.idProduct_H = HIBYTE(DEVICE_PRODUCT_ID),
.bcdDevice_L = 0,
.bcdDevice_H = 2,
.iManufacturer = 1,
.iProduct = 2,
.iSerialNumber = LAST_NUM_STRING_DESCR,
.bNumConfigurations =1};

const uint8_t aConfDescriptor[] = 
    {
// CONFIGURATION Descriptor  (Table 9-10 USB specification)
        0x09,               //bLength: Configuration Descriptor size
        USB_CFG_DESC_TYPE,  // bDescriptorType: Configuration
        41, 0,              // wTotalLength low & high  sizeof (configuration + interface + endpoint + HID) 34bytes
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
        0x02,               //bNumEndpoints
        USB_CLASS_HID,      //bInterfaceClass
        0x00,               //bInterfaceSubClass (0=not bootable or 1=bootable)
        0x00,               //bInterfaceProtocol (if not bootable than always=0)
        0x00,               //iInterface (no string Interface descriptor)
        
// HID descriptor (6.2.1 Device Class Definition for Human Interface Devices (HID) Version 1.11 )
        0x09,               //bLength
        USB_HID_DESC_TYPE,  //bDescriptorType: HID
        0x11,  0x01,        //bBCDHID low & high (ver 1.11)   
        0x00,               //bCountryCode (not Localisation)
        0x01,               //bNumDescriptors (follow 1 report descriptor)
        USB_REPORT_DESC_TYPE,   //bDescriptorType (report)
        198,0x00,  //wDescriptorLength (report descriptor lenth)
 
// ENDPOINT descriptor   (Table 9-13 USB specification) 
        0x07,                       //bLength:
        USB_EP_DESC_TYPE,           //bDescriptorType
        IN_ENDPOINT | 0x01,         //bEndpointAddress(ep#1 direction=IN)
        EP_TRANSFER_TYPE_INTERRUPT, // bmAttributes
        0x10, 0x00,                 // wMaxPacketSize: 16 Byte max 
        USB_EP_MIN_REPORT_INTERVAL, // bInterval: Polling Interval (32 ms)   
          
        0x07,                       //bLength:
        USB_EP_DESC_TYPE,           //bDescriptorType
        OUT_ENDPOINT | 0x01,        //bEndpointAddress(ep#1 direction=IN)
        EP_TRANSFER_TYPE_INTERRUPT, // bmAttributes
        0x10, 0x00,                 // wMaxPacketSize: 16 Byte max 
        USB_EP_MIN_REPORT_INTERVAL  // bInterval: Polling Interval (32 ms) 
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
    'P',0, 'i',0, 'r',0, 'o',0, 'm',0, 'e',0, 't',0, 'e',0, 'r',0 //Product
};

const uint8_t aStringDescriptors3[]=
{
    12,                 // bLength
    USB_STR_DESC_TYPE,  //bDescriptorType
    '0',0, '0',0, '0',0, '0',0, '0',0  //Product
};

static const uint8_t HID_Sensor_ReportDesc[198] =
{
  HID_USAGE_PAGE_SENSOR,
  HID_USAGE_SENSOR_TYPE_ENVIRONMENTAL_TEMPERATURE,
  HID_COLLECTION(Physical),

  //feature reports (xmit/receive)
  HID_USAGE_PAGE_SENSOR,
  
  HID_USAGE_SENSOR_PROPERTY_MINIMUM_REPORT_INTERVAL,
  HID_LOGICAL_MIN_8(0x20),
  HID_LOGICAL_MAX_16(0xFF,0xFF),
  HID_REPORT_SIZE(16),
  HID_REPORT_COUNT(1),
  HID_UNIT_EXPONENT(0),
  HID_FEATURE(Data_Var_Abs),

  HID_USAGE_SENSOR_PROPERTY_REPORT_INTERVAL,
  HID_LOGICAL_MIN_8(0x20),
  HID_LOGICAL_MAX_16(0xFF,0xFF),
  HID_REPORT_SIZE(16),
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

uint8_t* StringDescriptors[LAST_NUM_STRING_DESCR+1]=
{
  (uint8_t*) &aStringDescriptors0,
  (uint8_t*) &aStringDescriptors1,
  (uint8_t*) &aStringDescriptors2,
  (uint8_t*) &aStringDescriptors3
};

/* ****************************************************************************
See chapter 9.4.3 USB 2.0 specification
*SPacket - pointer to Setup Packet structure
**addr  - pointer to pointer descriptor address variable
*len - pointer to descriptor length variable
output value: 1 -if have descriptor or 0 if descriptor lost
*************************************************************************** */
uint8_t USB_GetDescriptor(USB_SetupPacket *SPacket, uint16_t **addr, uint16_t *len)
{
  USB_STR_DESCRIPTOR *pSTR;
  uint8_t retval = 1;
  uint16_t descSize;
  switch (SPacket->wValue.H)
  {
    case USB_DEVICE_DESC_TYPE:
      descSize=(sizeof(sDeviceDescriptor)< SPacket->wLength)? sizeof(sDeviceDescriptor) : SPacket->wLength;
      *addr=(uint16_t *)&sDeviceDescriptor;
      *len = descSize;
      break;

    case USB_CFG_DESC_TYPE:
    //use only one configuration, but wValue.L is ignored
      descSize=(sizeof(aConfDescriptor)< SPacket->wLength)? sizeof(aConfDescriptor) : SPacket->wLength;
      *addr=(uint16_t *)&aConfDescriptor;
      *len = descSize;
      break;

    case USB_STR_DESC_TYPE:
      if (SPacket->wValue.L > LAST_NUM_STRING_DESCR)
        retval = 0; 
        else
        {
          pSTR = (USB_STR_DESCRIPTOR *)StringDescriptors[SPacket->wValue.L];
          descSize=(pSTR->bLength < SPacket->wLength)? pSTR->bLength : SPacket->wLength;
          *addr=(uint16_t *)pSTR;
          *len = descSize;
        }
      break;
        
    case USB_DEVICE_QR_DESC_TYPE:
#ifdef QUALIFIER_DESCRIPTOR
      descSize=(sizeof(sQualDescriptor)< SPacket->wLength)? sizeof(sQualDescriptor) : SPacket->wLength;
      *addr=(uint16_t *)&sQualDescriptor;
      *len = descSize;
#else
     retval = 0;
#endif
      break;
       
    case USB_REPORT_DESC_TYPE:
      descSize=(sizeof(HID_Sensor_ReportDesc)< SPacket->wLength)? sizeof(HID_Sensor_ReportDesc) : SPacket->wLength;
      *addr=(uint16_t *)&HID_Sensor_ReportDesc;
      *len = descSize;
      break;    
        
    default:
      retval = 0;      
  }
  return (retval);
}

#ifdef SWO_USB_LOG
/* ****************************************************************************
This routine used for Logging Events
it return pointer on text string corresponding to Descriptor Name
*SPacket - pointer to Setup Packet structure
output value: pointer to string
*************************************************************************** */
char *Log_descriptorName(USB_SetupPacket *SPacket)
{
  char *retval;
  switch (SPacket->wValue.H)
  {
    case USB_DEVICE_DESC_TYPE:
      retval = "DEVICE";
      break;
      
    case USB_CFG_DESC_TYPE:
      retval = "CONFIG";
      break;
      
    case USB_STR_DESC_TYPE:
      retval = "STRING";
      break;
        
    case USB_DEVICE_QR_DESC_TYPE:
      retval = "QUALIF";
      break;
      
   case USB_REPORT_DESC_TYPE:
      retval = "REPORT";
      break;
      
    default:
      retval = "??";
      break;
  }
  return (retval);
}
#endif