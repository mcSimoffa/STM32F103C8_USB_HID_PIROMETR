#ifndef __STM32F103xb_USBDEF_H
#define __STM32F103xb_USBDEF_H
#include "stm32f10x.h"

#define LOBYTE(x) ((uint8_t)(x & 0x00FF))
#define HIBYTE(x) ((uint8_t)((x & 0xFF00) >> 8))

typedef struct 
{
    uint32_t EPR[8];
    uint32_t RESERVED[8];
    uint32_t CNTR;
    uint32_t ISTR;
    uint32_t FNR;
    uint32_t DADDR;
    uint32_t BTABLE;
} USB_TypeDef;


/* USB device FS */
#define USB_BASE              (APB1PERIPH_BASE + 0x00005C00UL) /*!< USB_IP Peripheral Registers base address */
#define USB_PMAADDR           (APB1PERIPH_BASE + 0x00006000UL) /*!< USB_IP Packet Memory Area base address */
#define USB                   ((USB_TypeDef *)USB_BASE)

typedef struct 
{
    uint16_t TX_Address;
    uint16_t _res0;
    uint16_t TX_Count;
    uint16_t _res1;
    uint16_t RX_Address;
    uint16_t _res2;
    uint16_t RX_Count;
    uint16_t _res3;
} USB_BDT;

typedef struct 
{
    uint16_t  Number;       // EP number
    uint16_t  Type;         // EP Type
    uint8_t   TX_Max;       // Max TX EP Buffer
    uint8_t   RX_Max;       // Max RT EP Buffer
    uint16_t *pTX_BUFF;     // TX Buffer pointer
    uint32_t  lTX;          // TX Data length
    uint16_t *pRX_BUFF;     // RX Buffer pointer
    uint32_t  lRX;          // RX Data length
    uint16_t  status;      //for GET_STATUS answer
} USB_EPinfo;
#define EP_STATUS_HALT_ON           0x01
#define EP_STATUS_HALT_OFF          0x00

#define  BDT_COUNTn_RX_Msk 0x3FF    //RM0008 site 650
#define  USB_COUNT_RX_NUM_BLOCK_Pos 10
#define  USB_COUNT_RX_BLSIZE_Pos    15
#define  USB_COUNT_RX_BLSIZE        (1<<USB_COUNT_RX_BLSIZE_Pos)

//Endpoint register  bit positions
#define EP_CTR_RX   0x8000  /* Correct RX Transfer */
#define EP_DTOG_RX  0x4000  /* RX Data Toggle */
#define EP_STAT_RX  0x3000  /* RX Status */
#define EP_SETUP    0x0800  /* EndPoint Setup */
#define EP_TYPE     0x0600  /* EndPoint Type */
#define EP_KIND     0x0100  /* EndPoint Kind */
#define EP_CTR_TX   0x0080  /* Correct TX Transfer */
#define EP_DTOG_TX  0x0040  /* TX Data Toggle */
#define EP_STAT_TX  0x0030  /* TX Status */
#define EP_EA       0x000F  /* EndPoint Address */
#define EP_MASK (EP_CTR_RX | EP_SETUP | EP_TYPE | EP_KIND | EP_CTR_TX | EP_EA) //all bits without toggled bits


#define EP_TYPE_MASK_Pos                    (9U)                           
#define EP_BULK                             0x00000000U                    /*!< EndPoint BULK */
#define EP_CONTROL                          0x00000200U                    /*!< EndPoint CONTROL */
#define EP_ISOCHRONOUS                      0x00000400U                    /*!< EndPoint ISOCHRONOUS */
#define EP_INTERRUPT                        0x00000600U                    /*!< EndPoint INTERRUPT */

//* EP_STAT_TX: TX Status */
#define TX_DISABLE 0x0000 /* Disabled */
#define TX_STALL 0x0010   /* Stalled */
#define TX_NAK 0x0020     /* NAKed */
#define TX_VALID 0x0030   /* Valid */

/* EP_STAT_RX: RX Status */
#define RX_DISABLE 0x0000 /* Disabled */
#define RX_STALL 0x1000   /* Stalled */
#define RX_NAK 0x2000     /* NAKed */
#define RX_VALID 0x3000   /* Valid */


/******************************************************************************/
/*                       ISTR interrupt events                                */
/******************************************************************************/
#define ISTR_CTR    (0x8000) /* Correct TRansfer (clear-only bit) */
#define ISTR_DOVR   (0x4000) /* DMA OVeR/underrun (clear-only bit) */
#define ISTR_ERR    (0x2000) /* ERRor (clear-only bit) */
#define ISTR_WKUP   (0x1000) /* WaKe UP (clear-only bit) */
#define ISTR_SUSP   (0x0800) /* SUSPend (clear-only bit) */
#define ISTR_RESET  (0x0400) /* RESET (clear-only bit) */
#define ISTR_SOF    (0x0200) /* Start Of Frame (clear-only bit) */
#define ISTR_ESOF   (0x0100) /* Expected Start Of Frame (clear-only bit) */
#define ISTR_DIR    (0x0010)  /* DIRection of transaction (read-only bit)  */
#define ISTR_EP_ID  (0x000F)  /* EndPoint IDentifier (read-only bit)  */
//--------------------------
typedef struct //(Table 9-2 USB 2.0 specification. site 248)
{
    uint8_t L : 8;
    uint8_t H : 8;
} USBLIB_WByte;
//standatr setup packet structure. (str 29 book)
typedef struct
{
    uint8_t         bmRequestType;
    uint8_t         bRequest;
    USBLIB_WByte    wValue;
    USBLIB_WByte    wIndex;
    uint16_t        wLength;
} USBLIB_SetupPacket;
//---------------------------
#define USB_REQUEST_TYPE            0x60 // bits 5..6 Table 9-2 USB 2.0 specification)
#define USB_REQUEST_STANDARD        0x00
#define USB_REQUEST_CLASS           0x20
#define USB_REQUEST_VENDOR          0x40

#define USB_REQUEST_RECIPIENT       0x1F // bits 1...4 Table 9-2 USB 2.0 specification)
#define USB_REQUEST_DEVICE          0x00
#define USB_REQUEST_INTERFACE       0x01
#define USB_REQUEST_ENDPOINT        0x02


// USB Standard Request Codes (Table 9-4 USB 2.0 specification)
#define USB_REQUEST_GET_STATUS               0
#define USB_REQUEST_CLEAR_FEATURE            1
#define USB_REQUEST_SET_FEATURE              3
#define USB_REQUEST_SET_ADDRESS              5
#define USB_REQUEST_GET_DESCRIPTOR           6
#define USB_REQUEST_SET_DESCRIPTOR           7
#define USB_REQUEST_GET_CONFIGURATION        8
#define USB_REQUEST_SET_CONFIGURATION        9
#define USB_REQUEST_GET_INTERFACE            10
#define USB_REQUEST_SET_INTERFACE            11
#define USB_REQUEST_SYNC_FRAME               12

//USB Class HID Request Codes (chapter 7.2 hid1_11.pdf specification)
#define USB_HID_GET_REPORT          0x01
#define USB_HID_GET_IDLE            0x02
#define USB_HID_GET_PROTOCOL        0x03
#define USB_HID_SET_REPORT          0x09
#define USB_HID_SET_IDLE            0x0A
#define USB_HID_SET_PROTOCOL        0x0B

// USB Descriptor Types (Table 9-5 USB 2.0 specification)
#define USB_DEVICE_DESC_TYPE        1
#define USB_CFG_DESC_TYPE           2
#define USB_STR_DESC_TYPE           3
#define USB_IFACE_DESC_TYPE         4
#define USB_EP_DESC_TYPE            5
#define USB_DEVICE_QR_DESC_TYPE     6
#define USB_OSPEED_CFG_DESC_TYPE    7
#define USB_IFACE_PWR_DESC_TYPE     8
// (chapter 7.1.1 site 49 hid1_11 specification)
#define USB_HID_DESC_TYPE           0x21
#define USB_REPORT_DESC_TYPE        0x22
#define USB_PHYSICAL_DESC_TYPE      0x23


// USB Device Classes (https://www.usb.org/defined-class-codes)
#define USB_CLASS_IN_INTERFACE_DESCRIPTOR   0x00
#define USB_CLASS_AUDIO                     0x01
#define USB_CLASS_COMM                      0x02
#define USB_CLASS_HID                       0x03
#define USB_CLASS_PHYSICAL                  0x05
#define USB_CLASS_IMAGE                     0x06
#define USB_CLASS_PRINTER                   0x07
#define USB_CLASS_MASS_STORAGE              0x08
#define USB_CLASS_HUB                       0x09
#define USB_CLASS_CDC_DATA                  0x0A
#define USB_CLASS_STMART_CARD               0x0B
#define USB_CLASS_CONTENT_SECURITY          0x0D
#define USB_CLASS_VIDEO                     0x0E
#define USB_CLASS_PERSONAL HEALTHCARE		0x0F
#define USB_CLASS_AUDIO_VIDEO_DEVICES		0x10
#define USB_CLASS_BILLBOARD_DEVICE			0x11
#define USB_CLASS_USB_TYPE_C_BRIDGE			0x12
#define USB_CLASS_DIAGNOSTIC_DEVICE			0xDC
#define USB_CLASS_WIRELESS_CONTROLLER		0xE0
#define USB_CLASS_MISCELLANEOUS				0xEF
#define USB_CLASS_APPLICATION_SPECIFIC		0xFE
#define USB_CLASS_VENDOR_SPECIFIC			0xFF

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

/*typedef struct // Table 9-9 USB specification
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
} Typedef_USB_DEVICE_QUALIFIER_DESCRIPTOR;*/

typedef struct
{
    uint8_t bLength;
    uint8_t bDescriptorType;
} USB_STR_DESCRIPTOR;

//configuration  descriptor field 'bmAttributes'  (Table 9-10 USB specification offset = 7)
#define BMATTRIBUTES_MASK           0x80
#define BMATTRIBUTES_SELF_POWERED   0x40
#define BMATTRIBUTES_REMOTE_WAKEUP  0x20

//configuration endpoint descriptor field 'bEndpointAddress' (Table 9-13 USB specification offset = 2)
#define IN_ENDPOINT                 0x80
#define OUT_ENDPOINT                0x00

//configuration endpoint descriptor field 'bmAttributes' (Table 9-13 USB specification offset = 3)
#define EP_TRANSFER_TYPE_CONTROL                  0x00
#define EP_TRANSFER_TYPE_ISOCHRONOUS              0x01
#define EP_TRANSFER_TYPE_BULK                     0x02
#define EP_TRANSFER_TYPE_INTERRUPT                0x03

#define DEVICE_VENDOR_ID 0x25AE
#define DEVICE_PRODUCT_ID 0x24AB

//GET_STATUS request (Figure 9-4 USB 2.0 specification)
#define STATUS_SELF_POWERED         0x01
#define STATUS_REMOTE_WAKEUP        0x02

#endif //__STM32F103xb_USBDEF_H