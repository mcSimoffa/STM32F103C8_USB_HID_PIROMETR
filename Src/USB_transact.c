#include "stm32f103xb_usbdef.h"
#include "USB_transact.h"
#include "additional_func.h"

extern char *pFloat;
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
 if ((pSetup->bmRequestType & USB_REQUEST_TYPE) == USB_REQUEST_STANDARD)	//Request type Standard ? 
  {
#ifdef SWOLOG
  pFloat = stradd (pFloat,"\r\nSTANDARD ");
#endif
  switch (pSetup->bRequest) 
  {
  case USB_REQUEST_GET_DESCRIPTOR:
  #ifdef SWOLOG
    pFloat = stradd (pFloat,"GET_DESCRIPTOR ");
    pFloat = stradd (pFloat,Log_descriptorName(pSetup));
  #endif
    uint16_t *descAddr;
    uint16_t desclen;
    if (USB_GetDescriptor(pSetup, &descAddr, &desclen))
      USBLIB_SendData(0, descAddr, desclen);
    else
    {
      BDTable[0].TX_Count = 0;
      USBLIB_setStatTx(0, TX_STALL);
    }
    break;
      
  case USB_REQUEST_SET_ADDRESS:
  #ifdef SWOLOG
    pFloat = stradd (pFloat,"SET_ADDRESS");
  #endif
    USBLIB_SendData(0, 0, 0);
    DeviceAddress = pSetup->wValue.L;
    break;

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
        USBLIB_SendData(0, &DeviceStatus, 2);
        break;
      case USB_REQUEST_INTERFACE:
      #ifdef SWOLOG
        pFloat = stradd (pFloat," IFACE");
      #endif
        USBLIB_SendData(0, &StatusVal, 2);
      case USB_REQUEST_ENDPOINT:
      #ifdef SWOLOG
        pFloat = stradd (pFloat," ENDP");
      #endif
        StatusVal = ((USB->EPR[pSetup->wIndex.L] & RX_VALID) == RX_STALL) ? 1:0;
        USBLIB_SendData(0, &StatusVal, 2);  
        break;
      default:
        USBLIB_SendData(0, 0, 0);
    }
  case USB_REQUEST_GET_CONFIGURATION:
  #ifdef SWOLOG
    pFloat = stradd (pFloat,"GET_CONF");
  #endif
    USBLIB_SendData(0, &DeviceConfigured, 1);                              //pochemu otpravka 1 byte a ne  2 ????????????                                                                      
    break;

  case USB_REQUEST_SET_CONFIGURATION:
  #ifdef SWOLOG
    pFloat = stradd (pFloat,"SET_CONF");
  #endif
    //if wValue.L= 0  State = Adresovano
    //else need STALL answer
    if (pSetup->wValue.L == 1)
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
    USBLIB_SendData(0, 0, 0);        // here need manipulate EP1 status
    break;
    
  default:
    asm("nop");
  #ifdef SWOLOG
    pFloat = stradd (pFloat,"??");
  #endif
  } //switch (pSetup->bRequest)
} //StandardRequestHandler
  
 return(1); 
  
  
}

/*
          StandardRequestHandler();
        
        else if ((pSetup->bmRequestType & USB_REQUEST_TYPE) == USB_REQUEST_CLASS)	
          ClassRequestHandler();





 if (((TR_pSetup->bmRequestType & USB_REQUEST_TYPE) == USB_REQUEST_CLASS) && (TR_pSetup->bRequest == USB_HID_SET_REPORT))
               switch(TR_pSetup->wValue.H)
                {
                  case USB_HID_REPORT_IN:
                  #ifdef SWOLOG
                    pFloat = stradd (pFloat," IN");
                  #endif
                    if (USB_Callback.SetInputReport)
                      USB_Callback.SetInputReport(EP0_Collect.pData + sizeof(USB_pSetup), EP0_Collect.cnt - sizeof(USB_pSetup));
                    break;
                    
                  case USB_HID_REPORT_OUT:
                  #ifdef SWOLOG
                    pFloat = stradd (pFloat," OUT");
                  #endif
                    if (USB_Callback.SetOutputReport)
                      USB_Callback.SetOutputReport(EP0_Collect.pData + sizeof(USB_pSetup), EP0_Collect.cnt - sizeof(USB_pSetup));
                    break;
                    
                  case USB_HID_REPORT_FEATURE: 
                  #ifdef SWOLOG
                    pFloat = stradd (pFloat," FEATURE");
                  #endif
                    if (USB_Callback.SetFeatureReport)
                      USB_Callback.SetFeatureReport(EP0_Collect.pData + sizeof(USB_pSetup), EP0_Collect.cnt - sizeof(USB_pSetup));
                    break;
        
                  default: 
                  #ifdef SWOLOG
                    pFloat = stradd (pFloat," ??");
                  #endif 
                    asm("nop");
                }
            USBLIB_SendData(0, 0, 0); //ACK packet send
            EP0_Collect.cnt = 0;

* ****************************************************************************
This inline routine runs if we have packet on EP=0 (control), 
Type of packet is SETUP, Type of request is Standard
***************************************************************************** 
__inline void StandardRequestHandler()
()

* ****************************************************************************
This inline routine runs if we have packet on EP=0 (control), 
Type of packet is SETUP, Type of request is Class
***************************************************************************** 
__inline void ClassRequestHandler()
{
#ifdef SWOLOG
  pFloat = stradd (pFloat," CLASS ");
#endif
  switch (pSetup->bRequest) 
  {
    case USB_HID_GET_REPORT:
    #ifdef SWOLOG
      pFloat = stradd (pFloat,"GET REPORT");
    #endif
      uint16_t *ppReportAddr;
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
            USB_Callback.GetInputReport(&ppReportAddr, &pReportSize);
            USBLIB_SendData(0, ppReportAddr, pReportSize);  
          }
          break;
          
        case USB_HID_REPORT_OUT:
        #ifdef SWOLOG
          pFloat = stradd (pFloat," OUT");
        #endif
          if (USB_Callback.GetOutputReport)
          {
            USB_Callback.GetOutputReport(&ppReportAddr, &pReportSize);
            USBLIB_SendData(0, ppReportAddr, pReportSize);  
          }
          break;
          
        case USB_HID_REPORT_FEATURE:
        #ifdef SWOLOG
          pFloat = stradd (pFloat," FEATURE");
        #endif
          if (USB_Callback.GetFeatureReport)
          {
            USB_Callback.GetFeatureReport(&ppReportAddr, &pReportSize);
            USBLIB_SendData(0, ppReportAddr, pReportSize);  
          }
          break;

        default: 
        #ifdef SWOLOG
          pFloat = stradd (pFloat," ??");
        #endif 
          asm("nop");
      } //switch(pSetup->wValue.H)
    break;
                          
    case USB_HID_SET_REPORT:
    #ifdef SWOLOG
      pFloat = stradd (pFloat,"SET REPORT ");
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
    reportPeriod = pSetup->wValue.H;
    USBLIB_SendData(0, 0, 0);
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
  } //switch (pSetup->bRequest) 
} //void ClassRequestHandler() */