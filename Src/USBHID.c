/* ******    TODO    *********
sdelat state mashine dlay SETs
sdelat callbacki dlya SETs: SET REPORT
sdelat otvety 0,0,0 nd SETs

*/
#include "USBHID.h"
#include "usb_descriptor.h"
#include <stdlib.h>
#include "Systick.h"
#include "additional_func.h"
#include "oringbuf.h"

#define EPCOUNT   2
__no_init volatile USB_BDT BDTable[EPCOUNT] @ USB_PMAADDR ; //Buffer Description Table

#ifdef SWOLOG
  void loggingSetupPacket(USB_SetupPacket *pSetup);
  void putlog();
  char debugBuf[512];
  char *pFloat;
#endif

USB_EPinfo EpData[EPCOUNT] =
{
  {.Number=0, .Type=EP_CONTROL,     .TX_Max=16, .RX_Max=16, .pTX_BUFF=0, .lTX=0, .pRX_BUFF=0, .lRX=0, .SendState=EP_SEND_READY, .ReadState=EP_READ_NOTHING},
  {.Number=1, .Type=EP_INTERRUPT,   .TX_Max=16, .RX_Max=16, .pTX_BUFF=0, .lTX=0, .pRX_BUFF=0, .lRX=0, .SendState=EP_SEND_READY, .ReadState=EP_READ_NOTHING}
};
uint8_t *pDataSetupPackets=0;     //pointer to Buffer accumulating all data packets for Setup transaction
uint16_t sizeDataSetupPackets=0;  //size of this Buffer
USB_SetupPacket   *SetupPacket;
uint8_t  DeviceAddress = 0;
uint16_t DeviceConfigured = 0;
uint8_t reportPeriod = 0;
uint16_t DeviceStatus = STATUS_BUS_POWERED;

Typedef_USB_Callback USB_Callback={
  .GetInputReport = 0,
  .GetOutputReport = 0,
  .GetFeatureReport = 0
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
      EpData[i].SendState = EP_SEND_READY;
      EpData[i].ReadState = EP_READ_NOTHING;
      }
    
    for (uint8_t i = EPCOUNT; i < 8; i++) //inactive endpoints
        USB->EPR[i] = i | RX_DISABLE | TX_DISABLE;
    
    DeviceConfigured = 0;
    USB->CNTR   = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_ERRM | USB_CNTR_SOFM;
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
      pFloat = stradd (pFloat, "Sp");
      putlog();
    #endif
      if (USB->DADDR & 0x7f) 
      {
        USB->DADDR = 0;
        USB->CNTR &= ~ USB_CNTR_SUSPM;
        USB->CNTR |= USB_CNTR_WKUPM;
      #ifdef SWOLOG  
        pFloat = stradd (debugBuf, "\r\nISTR=");
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
      if ((USB->EPR[0] & EP_STAT_TX) == TX_STALL) //this part turn off the STALL if it was be turned on before
        USBLIB_setStatTx(0, TX_NAK);
      
      for (int i=1;i<EPCOUNT;i++)
      {
        if(EpData[i].SendState == EP_SEND_INITIATE)
        {
          if (EpData[i].lTX > 0)
            USBLIB_EPBuf2Pma(i);
          else
            BDTable[i].TX_Count = 0;
          USBLIB_setStatTx(i, TX_VALID);
          EpData[i].SendState = EP_SEND_BUSY;
          putlog();
        } 
      }
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
        SetupPacket = (USB_SetupPacket *)EpData[EPn].pRX_BUFF;
      #ifdef SWOLOG
        pFloat = stradd (pFloat,"\r\nSETUP ");
        loggingSetupPacket(SetupPacket);
      #endif
        
        //it had been received SETUP packet with OUT wLength>0. Next will be wLength bytes datas
        if (((SetupPacket->bmRequestType & USB_REQUEST_DIR) == USB_REQUEST_DIR_OUT) && (SetupPacket->wLength >0))
        {
          uint16_t needMem = sizeof(SetupPacket)+ SetupPacket->wLength; //how many bytes need to save this transaction with SETUP packet
          if (sizeDataSetupPackets < needMem) //has too little allocated mem ?
          {
            free(pDataSetupPackets);
            pDataSetupPackets = 0;
            sizeDataSetupPackets = 0;
          }
            
          if (!pDataSetupPackets)
          {
            pDataSetupPackets = malloc(needMem);
            if(pDataSetupPackets)
              sizeDataSetupPackets = needMem;
            else
              while(1)
                asm("nop"); //memory didn't provide
          }
   tyjtyjtyjtj;
        } 
        
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
            pFloat = stradd (pFloat,Log_descriptorName(SetupPacket));
          #endif
            uint16_t *descAddr;
            uint16_t desclen;
            if (USB_GetDescriptor(SetupPacket, &descAddr, &desclen))
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
            DeviceAddress = SetupPacket->wValue.L;
            break;

          case USB_REQUEST_GET_STATUS:
          #ifdef SWOLOG
            pFloat = stradd (pFloat,"GET_STATUS");
          #endif
            uint16_t StatusVal;
            StatusVal = 0;
            switch(SetupPacket->bmRequestType & USB_REQUEST_RECIPIENT)
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
                StatusVal = ((USB->EPR[SetupPacket->wIndex.L] & RX_VALID) == RX_STALL) ? 1:0;
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
            USBLIB_SendData(0, 0, 0);        // here need manipulate EP1 status
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
              pFloat = stradd (pFloat,"GET REPORT");
            #endif
              uint16_t *ppReportAddr;
              uint16_t pReportSize;
              pReportSize = SetupPacket->wLength;
              switch(SetupPacket->wValue.H)
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
                  pFloat = stradd (pFloat," FEATURE");
                #endif 
                  asm("nop");
              }
            break;
                                  
            case USB_HID_SET_REPORT:
            #ifdef SWOLOG
              pFloat = stradd (pFloat,"SET REPORT");
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
          }//switch (SetupPacket->bRequest) 
        }//Request type Class ?
      }//Setup packet
      else
      {
      #ifdef SWOLOG
        pFloat = printHexMem(EpData[EPn].pRX_BUFF,pFloat,EpData[EPn].lRX);
      #endif  
      }
    }//Control endpoint 
    else 
      { // Got data from another EP
        asm("nop");  // Call user function
      #ifdef SWOLOG
        pFloat = stradd (pFloat,"\r\nGot Data ");
        pFloat = printHexMem(EpData[EPn].pRX_BUFF,pFloat,EpData[EPn].lRX);
      #endif         
        USBLIB_SendData(EPn, 0, 0);                   //nujno li podtverjdat prinatie dannie
      }// Got data from another EP
                                                                      
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
          if (EpData[EPn].SendState == EP_SEND_BUSY)
            EpData[EPn].SendState = EP_SEND_READY;
          }
      }//something transmitted
}//void USB_EPHandler

//*****************************************************************************
void USBLIB_SendData(uint8_t EPn, uint16_t *Data, uint16_t Length)
{
  USB->EPR[EPn] &= 0x870f;   //reset flag CTR_TX          //vozmojno zdes nado obnulit flag TC
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
  pFloat = stradd (pFloat, "\r\nPMA[");
  pFloat = itoa(EPn ,pFloat,10,0);
  pFloat = stradd (pFloat, "]->Buffer ");
  pFloat = itoa(Count ,pFloat,10,0);
  pFloat = stradd (pFloat, " bytes ");
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
  pFloat = stradd (pFloat, "\r\nBuffer->PMA[");
  pFloat = itoa(EPn ,pFloat,10,0);
  pFloat = stradd (pFloat, "] ");
  pFloat = itoa(Count ,pFloat,10,0);
  pFloat = stradd (pFloat, " bytes ");
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

/* ****************************************************************************
This routine try to send IN REPORT. 
EPn - endpoint number to use
Data - pointer to outgong data
Length - length data in bytes
return value: 0 - if data deny to send. 1-Data allow to send and will be send
**************************************************************************** */
uint8_t USB_sendReport(uint8_t EPn, uint16_t *Data, uint16_t Length)
{ 
  if ((EpData[EPn].SendState == EP_SEND_READY) && DeviceConfigured)
  {
    EpData[EPn].lTX = Length;
    EpData[EPn].pTX_BUFF = Data;
    EpData[EPn].SendState = EP_SEND_INITIATE;
    return(1);
  }
  return(0);
}

#ifdef SWOLOG
/* ****************************************************************************
SWO logging for SETUP packet type to SWO by ITM_SendChar
pSetup - pointer on struct
**************************************************************************** */
void loggingSetupPacket(USB_SetupPacket *pSetup)
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
