#include "stm32f103xb_usbdef.h"
#include "USB_transact.h"
#include "USB_sharedFunctions.h"
#include "usb_descriptor.h"
#include "additional_func.h"
#include "usb_callback.h"

#ifdef SWOLOG
  extern char *pFloat;
#endif
uint16_t DeviceStatus = STATUS_BUS_POWERED;
uint8_t DeviceConfigured = 0;
uint8_t IdleRate = 0;

Typedef_USB_Callback USB_Callback={
  .GetInputReport = 0,
  .GetOutputReport = 0,
  .GetFeatureReport = 0,
  .SetInputReport = 0, 
  .SetOutputReport = 0,
  .SetFeatureReport = 0
};

/* ***************************************************************************
This routine does parcing  request  for IN direction Data stage 
pSetup      - pointer on location SETUP packet
ppDataToIn  - double pointer on Data buffer to send in IN DATA stage
sizeData    - pointer to size DATA stage
return Value:
              0 if TX_STALL,  or 1 if TX_VALID
              It also set for *ppDataToIn and sizeData  a relevants values
****************************************************************************** */
uint8_t USB_IN_requestHandler(USB_SetupPacket *pSetup, uint16_t **ppDataToIn, uint16_t *sizeData)
{
  uint8_t retval = 1;
 if ((pSetup->bmRequestType & USB_REQUEST_TYPE) == USB_REQUEST_STANDARD)	//Request type Standard ? 
  {
#ifdef SWOLOG
  pFloat = stradd (pFloat,"\r\nSTANDARD ");
#endif
  switch (pSetup->bRequest) 
  {  
    case USB_REQUEST_GET_CONFIGURATION:
  #ifdef SWOLOG
    pFloat = stradd (pFloat,"GET_CONF");
  #endif
    *ppDataToIn = (uint16_t *)&DeviceConfigured;
    *sizeData = 1;                                                                     
    break;  //USB_REQUEST_GET_CONFIGURATION
    
  case USB_REQUEST_GET_DESCRIPTOR:
  #ifdef SWOLOG
    pFloat = stradd (pFloat,"GET_DESCRIPTOR ");
    pFloat = stradd (pFloat,Log_descriptorName(pSetup));
  #endif
    uint16_t *descAddr;
    uint16_t desclen;
    if (USB_GetDescriptor(pSetup, &descAddr, &desclen))
    {
      *ppDataToIn = descAddr;
      *sizeData = desclen;
    } else
        retval = 0; //STALL
    break;  //USB_REQUEST_GET_DESCRIPTOR

  case USB_REQUEST_GET_INTERFACE:
  #ifdef SWOLOG
    pFloat = stradd (pFloat,"GET_INTFACE");
  #endif
    uint8_t InterfaceNumber;
    InterfaceNumber = 0;
    *ppDataToIn = (uint16_t *)&InterfaceNumber;
    *sizeData = 1;
    break;  //USB_REQUEST_GET_INTERFACE
    
  case USB_REQUEST_GET_STATUS:
  #ifdef SWOLOG
    pFloat = stradd (pFloat,"GET_STATUS");
  #endif
    uint16_t StatusVal;
    StatusVal = 0;
    switch(pSetup->bmRequestType & USB_REQUEST_RECIPIENT)
    {
      case USB_REQUEST_DEVICE:
      #ifdef SWOLOG
        pFloat = stradd (pFloat," DEVICE");
      #endif
        *ppDataToIn = &DeviceStatus;
        *sizeData = 2;
        break;
        
      case USB_REQUEST_INTERFACE:
      #ifdef SWOLOG
        pFloat = stradd (pFloat," IFACE");
      #endif
        *ppDataToIn = &StatusVal;
        *sizeData = 2;
        break;
        
      case USB_REQUEST_ENDPOINT:
      #ifdef SWOLOG
        pFloat = stradd (pFloat," ENDP");
      #endif
        StatusVal = USB_geStatusEP(pSetup->wIndex.L);
        *ppDataToIn = &StatusVal;
        *sizeData = 2;  
        break;
        
      default:
        *sizeData = 0;  //ZLP
    }
    break;  //USB_REQUEST_GET_STATUS   
  } //switch (pSetup->bRequest)
 }  //Request type Standard 
 
  else if ((pSetup->bmRequestType & USB_REQUEST_TYPE) == USB_REQUEST_CLASS)
    {
     switch (pSetup->bRequest) 
      {
        case USB_HID_GET_REPORT:
        #ifdef SWOLOG
          pFloat = stradd (pFloat,"GET REPORT");
        #endif
          uint16_t *pReportAddr;
          uint16_t pReportSize;
          pReportSize = pSetup->wLength;
          switch(pSetup->wValue.H)
          {
            case USB_HID_REPORT_IN:
            #ifdef SWOLOG
              pFloat = stradd (pFloat," IN");
            #endif
              if (USB_Callback.GetInputReport)
              {
                USB_Callback.GetInputReport(pSetup->wValue.L, &pReportAddr, &pReportSize);
                *ppDataToIn = pReportAddr;
                *sizeData = pReportSize;  
              }
              else
                retval = 0; //STALL
              break;  //USB_HID_REPORT_IN
              
            case USB_HID_REPORT_OUT:
            #ifdef SWOLOG
              pFloat = stradd (pFloat," OUT");
            #endif
              if (USB_Callback.GetOutputReport)
              {
                USB_Callback.GetOutputReport(pSetup->wValue.L, &pReportAddr, &pReportSize);
                *ppDataToIn = pReportAddr;
                *sizeData = pReportSize;  
              }
              else
                retval = 0; //STALL
              break;  //USB_HID_REPORT_OUT
              
            case USB_HID_REPORT_FEATURE:
            #ifdef SWOLOG
              pFloat = stradd (pFloat," FEATURE");
            #endif
              if (USB_Callback.GetFeatureReport)
              {
                USB_Callback.GetFeatureReport(pSetup->wValue.L, &pReportAddr, &pReportSize);
                *ppDataToIn = pReportAddr;
                *sizeData = pReportSize;  
              }
              else
                retval = 0; //STALL
              break;

            default: 
            #ifdef SWOLOG
              pFloat = stradd (pFloat," ??");
            #endif 
              asm("nop");
          } //switch(pSetup->wValue.H)
          break;  //USB_HID_GET_REPORT
                                                        
        case USB_HID_GET_IDLE:
        #ifdef SWOLOG
          pFloat = stradd (pFloat,"Get Idle");
        #endif
          *ppDataToIn = (uint16_t*)&IdleRate;
          *sizeData = 1;          
          break; //USB_HID_GET_IDLE
                                                        
        case USB_HID_GET_PROTOCOL:
        #ifdef SWOLOG
          pFloat = stradd (pFloat,"Get Protocol");
        #endif 
          retval = 0; // Not boot Device. Not supported (STALL)
          break;
                                          
        default:
        #ifdef SWOLOG
          pFloat = stradd (pFloat,"??");
        #endif  
        break;
      } //switch (pSetup->bRequest) 
  } //void ClassRequestHandler
 return(retval);  
}


/* ***************************************************************************
This routine does parcing  request  for OUT direction Data stage 
pSetup      - pointer on location SETUP packet
pDataOut    - pointer on received Data in DATA stage
sizeData    - size DATA stage
return Value:
              0 if TX_STALL,  or 1 if TX_VALID
****************************************************************************** */
uint8_t USB_OUT_requestHandler(USB_SetupPacket *pSetup, uint16_t *pDataOut, uint16_t sizeData)
{
  uint8_t retval = 1;
  if ((pSetup->bmRequestType & USB_REQUEST_TYPE) == USB_REQUEST_STANDARD)	//Request type Standard ? 
  {
  #ifdef SWOLOG
    pFloat = stradd (pFloat,"\r\nSTANDARD ");
  #endif
    switch (pSetup->bRequest) 
    {
      case USB_REQUEST_CLEAR_FEATURE:
      #ifdef SWOLOG
         pFloat = stradd (pFloat,"CLEAR_FEATURE");
      #endif
                                                          // here need manipulate EP1 status       
        break;  //USB_REQUEST_CLEAR_FEATURE
    
      case USB_REQUEST_SET_ADDRESS:
    #ifdef SWOLOG
      pFloat = stradd (pFloat,"SET_ADDRESS");
    #endif 
      USB_setAddress(pSetup->wValue.L);
      break;  //USB_REQUEST_SET_ADDRESS
    
      case USB_REQUEST_SET_CONFIGURATION:
    #ifdef SWOLOG
      pFloat = stradd (pFloat,"SET_CONF");
    #endif
      if (pSetup->wValue.L == 1)
        DeviceConfigured = 1;   //if only one configuration - allowed so
      break;
      
      case USB_REQUEST_SET_FEATURE:
    #ifdef SWOLOG
      pFloat = stradd (pFloat,"SET_FEATURE");
    #endif 
                                                             // here need manipulate EP1 status
      break;  //USB_REQUEST_SET_FEATURE
    
    default:
      asm("nop");
    #ifdef SWOLOG
      pFloat = stradd (pFloat,"??");
    #endif
    } //switch (pSetup->bRequest)
  } //Request type Standard
  else if ((pSetup->bmRequestType & USB_REQUEST_TYPE) == USB_REQUEST_CLASS)
    {
   #ifdef SWOLOG
    pFloat = stradd (pFloat,"\r\nCLASS HID");
  #endif
      switch (pSetup->bRequest)
      {
        case USB_HID_SET_IDLE:
        #ifdef SWOLOG
          pFloat = stradd (pFloat,"Set Idle");
        #endif
          IdleRate = pSetup->wValue.H;
          break;

        case USB_HID_SET_PROTOCOL:
        #ifdef SWOLOG
          pFloat = stradd (pFloat,"Set protocol");
        #endif  
          break;
        
        case USB_HID_SET_REPORT:
        #ifdef SWOLOG
          pFloat = stradd (pFloat,"Set Report ");
        #endif
          switch(pSetup->wValue.H)
          {
            case USB_HID_REPORT_IN:
            #ifdef SWOLOG
              pFloat = stradd (pFloat,"IN");
            #endif
              if (USB_Callback.SetInputReport)
                USB_Callback.SetInputReport(pSetup->wValue.L, (uint8_t *)pDataOut, sizeData);
              break;
              
            case USB_HID_REPORT_OUT:
            #ifdef SWOLOG
              pFloat = stradd (pFloat,"OUT");
            #endif
              if (USB_Callback.SetOutputReport)
                USB_Callback.SetOutputReport(pSetup->wValue.L, (uint8_t *)pDataOut, sizeData);
              break;
              
            case USB_HID_REPORT_FEATURE: 
            #ifdef SWOLOG
              pFloat = stradd (pFloat,"FEATURE");
            #endif
              if (USB_Callback.SetFeatureReport)
                USB_Callback.SetFeatureReport(pSetup->wValue.L, (uint8_t *)pDataOut, sizeData);
              break;

            default: 
            #ifdef SWOLOG
              pFloat = stradd (pFloat,"???");
            #endif 
              asm("nop");
          } //switch pSetup->wValue.H (IN OUT FEATURE)
      } //switch pSetup->bRequest 
    } //USB_REQUEST_CLASS
  return (retval);
}

void USB_setDeviceCofig(uint16_t val)
{DeviceConfigured = val;}

uint16_t  USB_getDeviceConfig()
{return(DeviceConfigured);}