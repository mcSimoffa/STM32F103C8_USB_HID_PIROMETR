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
} USB_EPinfo;
#define BDT_COUNTn_RX_Msk 0x3FF     //RM0008 site 650

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
typedef struct
{
    uint8_t Recipient : 5;
    uint8_t Type : 2;
    uint8_t Dir : 1;
} USBLIB_RequestType;

typedef struct
{
    uint8_t L : 8;
    uint8_t H : 8;
} USBLIB_WByte;
//standatr setup packet structure. (str 29 book)
typedef struct
{
    USBLIB_RequestType bmRequestType;
    uint8_t            bRequest;
    USBLIB_WByte       wValue;
    USBLIB_WByte       wIndex;
    uint8_t            wLength;
} USBLIB_SetupPacket;
//---------------------------


// USB Standard Request Codes (Table 9-4 USB 2.0 specification)
#define USB_REQUEST_GET_STATUS 0
#define USB_REQUEST_CLEAR_FEATURE 1
#define USB_REQUEST_SET_FEATURE 3
#define USB_REQUEST_SET_ADDRESS 5
#define USB_REQUEST_GET_DESCRIPTOR 6
#define USB_REQUEST_SET_DESCRIPTOR 7
#define USB_REQUEST_GET_CONFIGURATION 8
#define USB_REQUEST_SET_CONFIGURATION 9
#define USB_REQUEST_GET_INTERFACE 10
#define USB_REQUEST_SET_INTERFACE 11
#define USB_REQUEST_SYNC_FRAME 12

// USB Descriptor Types (Table 9-5 USB 2.0 specification)
#define USB_DEVICE_DESC_TYPE 1
#define USB_CFG_DESC_TYPE 2
#define USB_STR_DESC_TYPE 3
#define USB_IFACE_DESC_TYPE 4
#define USB_EP_DESC_TYPE 5
#define USB_DEVICE_QR_DESC_TYPE 6
#define USB_OSPEED_CFG_DESC_TYPE 7
#define USB_IFACE_PWR_DESC_TYPE 8

/* USB Device Classes */
#define USB_RESERVED 0x00
#define USB_AUDIO 0x01
#define USB_COMM 0x02
#define USB_HID 0x03
#define USB_MONITOR 0x04
#define USB_PHYSIC 0x05
#define USB_POWER 0x06
#define USB_PRINTER 0x07
#define USB_STORAGE 0x08
#define USB_HUB 0x09
#define USB_VENDOR_SPEC 0xFF


#define DEVICE_VENDOR_ID 0x25AE
#define DEVICE_PRODUCT_ID 0x24AB

// USB Descriptor Types (Table 9-5 USB 2.0 specification)
#define USB_DEVICE_DESC_TYPE 1
#define USB_CFG_DESC_TYPE 2
#define USB_STR_DESC_TYPE 3
#define USB_IFACE_DESC_TYPE 4
#define USB_EP_DESC_TYPE 5
#define USB_DEVICE_QR_DESC_TYPE 6
#define USB_OSPEED_CFG_DESC_TYPE 7
#define USB_IFACE_PWR_DESC_TYPE 8

typedef struct _USB_STRING_DESCRIPTOR_ 
{
    uint8_t bLength;
    uint8_t bDescriptorType;
} USB_STR_DESCRIPTOR;

#endif //__STM32F103xb_USBDEF_H