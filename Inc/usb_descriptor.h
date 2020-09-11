#ifndef _USB_DESCRIPTOR_H_
 #define _USB_DESCRIPTOR_H_
 #include "stm32f10x.h"
 #include "stm32f103xb_usbdef.h"
 #include "HidSensorSpec.h"

// #define QUALIFIER_DESCRIPTOR     //if you need QUALIFIER DESCRIPTOR

typedef struct // Table 9-8 USB specification
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bcdUSB_L;
    uint8_t  bcdUSB_H;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint8_t  idVendor_L;
    uint8_t  idVendor_H;
    uint8_t  idProduct_L;
    uint8_t  idProduct_H;
    uint8_t  bcdDevice_L;
    uint8_t  bcdDevice_H;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} Typedef_USB_DEVICE_DESCRIPTOR;

#ifdef QUALIFIER_DESCRIPTOR
typedef struct // Table 9-9 USB specification
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bcdUSB_L;
    uint8_t  bcdUSB_H;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint8_t  bNumConfigurations;
    uint8_t  bReservedFuture;
} Typedef_USB_DEVICE_QUALIFIER_DESCRIPTOR;
#endif

typedef struct
{
    uint8_t bLength;
    uint8_t bDescriptorType;
} USB_STR_DESCRIPTOR;

 uint8_t USB_GetDescriptor(USB_SetupPacket *SPacket, uint16_t **addr, uint16_t *len);
#ifdef SWO_USB_LOG
 char *Log_descriptorName(USB_SetupPacket *SPacket);
#endif
#endif //_USB_DESCRIPTOR_H_