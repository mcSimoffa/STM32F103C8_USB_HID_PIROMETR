#include <stdlib.h>
#include <string.h>
#include "USB_packet.h"
#include "usb_interface.h"
#include "USB_sharedFunctions.h"
#include "USB_transact.h"
#include "Systick.h"
#include "additional_func.h"
#include "SWOfunction.h"

#define EPCOUNT   2
__no_init volatile USB_BDT BDTable[EPCOUNT] @ USB_PMAADDR ; //Buffer Description Table

#ifdef SWO_USB_LOG
  char *loggingSetupPacket(USB_SetupPacket *pSetup, char *pdst);
  char debugBuf[768];
  char *pFloat;
#endif

USB_EPinfo EpData[EPCOUNT] =
{
  {.Number=0, .Type=EP_CONTROL,     .TX_Max=16, .RX_Max=16, .pTX_BUFF=NULL, .lTX=0, .pRX_BUFF=NULL, .lRX=0, .SendState=EP_SEND_READY},
  {.Number=1, .Type=EP_INTERRUPT,   .TX_Max=16, .RX_Max=16, .pTX_BUFF=NULL, .lTX=0, .pRX_BUFF=NULL, .lRX=0, .SendState=EP_SEND_READY}
};

Typedef_OUT_TransactionCollector EP0_Collect=
{
  .cnt = 0,
  .allocSize = 0,
  .pData = NULL,
  .state = OUT_COLLECTOR_NOTHING
};

USB_SetupPacket   SetupPacketStorage;
USB_SetupPacket   *SetupPacket;
uint8_t  DeviceAddress = 0;
/* *****************************************************************************
 Main USB Interrupt Handler
***************************************************************************** */
void USB_LP_CAN1_RX0_IRQHandler()
{
    DWT->CYCCNT = 0;
    uint16_t istr=(uint16_t)USB->ISTR;
  #ifdef SWO_USB_LOG
    pFloat = stradd (debugBuf, "\r\n");  //new debugging string
  #endif
      
    if (USB->ISTR & USB_ISTR_SUSP) 
    {
      USB->ISTR &= ~USB_ISTR_SUSP;
      if (USB->DADDR & USB_DADDR_ADD) 
      {
        USB->DADDR = 0;
        USB->CNTR &= ~ USB_CNTR_SUSPM;
        USB->CNTR |= USB_CNTR_WKUPM;
      #ifdef SWO_USB_LOG 
        pFloat = stradd (pFloat, "\n---------\r\nTime=");
        pFloat = itoa(GetTick(), pFloat,10,0);
        pFloat = stradd (pFloat, "\r\nISTR=");
        pFloat = itoa(istr, pFloat,2,0);
        pFloat = stradd (pFloat, "\r\nFNR=");
        pFloat = itoa(USB->FNR ,pFloat,2,0);
        pFloat = stradd (pFloat, "\r\nEP0=");
        pFloat = itoa(USB->EPR[0] ,pFloat,2,0);
      #endif  
      }
    #ifdef SWO_USB_LOG
      pFloat = stradd (pFloat, " S");
      putlog(debugBuf, pFloat);
    #endif
      return;
    }
    
  #ifdef SWO_USB_LOG
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
    #ifdef SWO_USB_LOG
      pFloat = stradd (pFloat, "\r\nRESET");
      putlog(debugBuf, pFloat);
    #endif      
      USB_Reset();
      return;
    }
    if (USB->ISTR & USB_ISTR_CTR) 
    { //Handle data on EP
      USB->ISTR &= ~USB_ISTR_CTR;
      USB_EPHandler((uint16_t)USB->ISTR);
      putlog(debugBuf, pFloat);
      return;
    }
    
    if (USB->ISTR & USB_ISTR_ERR) 
    {
      USB->ISTR &= ~USB_ISTR_ERR;
  #ifdef SWO_USB_LOG
      pFloat = stradd (pFloat, "\r\nERR");
      putlog(debugBuf, pFloat);
  #endif
      return;
    }
    
    if (USB->ISTR & USB_ISTR_WKUP) 
    {      
      USB->ISTR &= ~USB_ISTR_WKUP;
      USB->CNTR &= ~ USB_CNTR_WKUPM;
      USB->CNTR |= USB_CNTR_SUSPM;
    #ifdef SWO_USB_LOG
      pFloat = stradd (pFloat, "\r\nWKUP");
      putlog(debugBuf, pFloat);       
    #endif
      return;
    }
    
    if (USB->ISTR & USB_ISTR_SOF) 
    {
      USB->ISTR &= ~USB_ISTR_SOF;
    #ifdef SWO_USB_LOG
      uint8_t printLogFlag = 0;
    #endif
      if ((USB->EPR[0] & EP_STAT_TX) == TX_STALL) //this part turn off the STALL if it was be turned on before
      {
        USBLIB_setStatTx(0, TX_NAK);
      #ifdef SWO_USB_LOG
        pFloat = stradd (pFloat, "\r\nSTALL -> NAK");
        printLogFlag = 1;
      #endif
      }
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
          printLogFlag = 1;
        } 
      }
    #ifdef SWO_USB_LOG
      if (printLogFlag)
        putlog(debugBuf, pFloat);
    #endif
      return;
    }   
    USB->ISTR = 0; //other interrupt
}

/* *****************************************************************************
 USb Reset Handler
***************************************************************************** */
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
          if(EpData[i].pRX_BUFF == NULL)
            exceptionFail();         //memory not provided
        #endif   
        }
      USB->EPR[i] = (EpData[i].Number | EpData[i].Type | RX_VALID | TX_NAK);
      EpData[i].SendState = EP_SEND_READY;
      }
    EP0_Collect.cnt = 0;
    EP0_Collect.state = OUT_COLLECTOR_NOTHING;
    for (uint8_t i = EPCOUNT; i < 8; i++) //inactive endpoints
        USB->EPR[i] = i | RX_DISABLE | TX_DISABLE;
    
    USB_setDeviceCofig (0);
    USB->CNTR   = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_ERRM | USB_CNTR_SOFM;
    USB->ISTR   = 0x00;
    USB->BTABLE = 0x00;
    USB->DADDR  = USB_DADDR_EF; //Enable USB device and set ZERO address
}

/* *****************************************************************************
 USB Endpoint Interrupt Handler
***************************************************************************** */
void USB_EPHandler(uint16_t Status)
{
    uint8_t  EPn = Status & ISTR_EP_ID; //endpoint number where occured
    uint32_t EP  = USB->EPR[EPn];
    uint8_t needAnswer = 0; //flag need ZLP or initiate send data
  #ifdef SWO_USB_LOG
    pFloat = stradd (pFloat,"\r\nEP=");
    pFloat = itoa(EP ,pFloat,2,0);
  #endif
    if (EP & EP_CTR_RX)     //something received ?
    {
      USB->EPR[EPn] &= 0x78f; //reset flag CTR_RX
    #ifdef SWO_USB_LOG
      pFloat = stradd (pFloat,"\r\nReceived in EP[");
      pFloat = itoa(EPn ,pFloat,10,0);
      pFloat = stradd (pFloat,"]: ");
    #endif
      USBLIB_Pma2EPBuf(EPn);  //move incoming to EP Buffer
      if (EpData[EPn].lRX > 0)  //have useful packet (non ZLP ACK) - need answer something
          needAnswer = 1;
      if (EPn == 0)       //Control endpoint ?
      { 
        if (EP & USB_EP0R_SETUP) 
        {
        SetupPacket = (USB_SetupPacket *) memcpy (&SetupPacketStorage, EpData[EPn].pRX_BUFF, sizeof(USB_SetupPacket)); //reserve copy SETUP packet
        EP0_Collect.state = OUT_COLLECTOR_NOTHING;
      #ifdef SWO_USB_LOG
        pFloat = stradd (pFloat,"\r\nSETUP ");
        pFloat=loggingSetupPacket(SetupPacket,pFloat);
      #endif
        
        //it had been received SETUP packet with OUT direction in DATA phase
        if ((SetupPacket->bmRequestType & USB_REQUEST_DIR) == USB_REQUEST_DIR_OUT)
        {
          EP0_Collect.expectedSize = SetupPacket->wLength;     //how many bytes need to save expected Data packets
          if (EP0_Collect.expectedSize) //it needs memory for OUT DATA stage ?
          {
          #ifdef SWO_USB_LOG
            pFloat = stradd (pFloat,"\r\nAssembing transaction");
          #endif
            if (EP0_Collect.allocSize < EP0_Collect.expectedSize) //has too little allocated mem ?
            {
              free(EP0_Collect.pData);
              EP0_Collect.pData = NULL;
              EP0_Collect.allocSize = 0;
            }          
            if (!EP0_Collect.pData)
            {
              EP0_Collect.pData = (uint8_t*)malloc(EP0_Collect.expectedSize);
              if(EP0_Collect.pData)
                EP0_Collect.allocSize = EP0_Collect.expectedSize;
              else
                exceptionFail(); //memory didn't provide
            }
            needAnswer = 0; //Answer ZLP will be send after whole transaction (after DATA stage)
          }
          EP0_Collect.cnt = 0;
          EP0_Collect.state = OUT_COLLECTOR_ASSEMB;
        } 
        else  //it had been received SETUP packet with IN direction in DATA phase
        {
          uint16_t *pBufForSend;
          uint16_t sizeForSend;
          //CALL parser transaction with IN DATA stage
          if (USB_IN_requestHandler(SetupPacket, &pBufForSend, &sizeForSend))
            USBLIB_SendData(0, pBufForSend, sizeForSend);
          else
          {
            BDTable[0].TX_Count = 0;
            USBLIB_setStatTx(0, TX_STALL);
          }
          needAnswer = 0;
        }        
      } //Setup packet
      else  // non SETUP packet
      { 
        if (EP0_Collect.state == OUT_COLLECTOR_ASSEMB) //assembling transaction in process ?
        {
          uint16_t remainingSize = EP0_Collect.expectedSize - EP0_Collect.cnt;
          uint16_t sizeToAdd = (remainingSize > EpData[0].lRX) ? EpData[0].lRX : remainingSize; //it's protection allocated memory  owerflow
          if (sizeToAdd)
          {
            memcpy(EP0_Collect.pData + EP0_Collect.cnt, EpData[0].pRX_BUFF, sizeToAdd);
            EP0_Collect.cnt += sizeToAdd;
          }
        }
      } // non SETUP packet
      
      // check: SETUP packet with OUT DATA stage direction is been collected ?
      if ((EP0_Collect.state == OUT_COLLECTOR_ASSEMB) &&(EP0_Collect.cnt >= EP0_Collect.expectedSize))
        {
          EP0_Collect.state = OUT_COLLECTOR_NOTHING;
          //CALL parser transaction with out DATA stage
          if (!USB_OUT_requestHandler(SetupPacket, (uint16_t*)EP0_Collect.pData, EP0_Collect.cnt))
            {
              BDTable[0].TX_Count = 0;
              USBLIB_setStatTx(0, TX_STALL);
              needAnswer = 0;
            }   
        }
    } //Control endpoint
    
    else 
      { // Got data from another EP
        asm("nop");  // Call user function
      #ifdef SWO_USB_LOG
        pFloat = stradd (pFloat,"\r\nGot Data an another EP ");
      #endif         
      }// Got data from another EP
      USBLIB_setStatRx(EPn, RX_VALID);
      if (needAnswer)
        {
          needAnswer = 0;
          USBLIB_SendData(EPn, 0, 0); //ACK
        }                                                                
    } //something received
    
    if (EP & EP_CTR_TX) //something transmitted
      {
        USB->EPR[EPn] &= 0x870f;   //reset flag CTR_TX
      #ifdef SWO_USB_LOG
        pFloat = stradd (pFloat, "\r\nTX complete EP[");
        pFloat = itoa(EPn ,pFloat,10,0);
        pFloat = stradd (pFloat, "] ");
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
          #ifdef SWO_USB_LOG
            pFloat = stradd (pFloat," End TX ");
          #endif
          if (EpData[EPn].SendState == EP_SEND_BUSY)
            EpData[EPn].SendState = EP_SEND_READY;
          USB_EPtransmitDone(EPn);
          }
      }//something transmitted
}//void USB_EPHandler
        
        
/* *****************************************************************************
 This routine initiate Send data. If datasize > EP size it is send first packet
and stay in queue remaining data
EPn     - endpoint number
Data    - pointer to source Data Array
Length  - size of Data Array
***************************************************************************** */
void USBLIB_SendData(uint8_t EPn, uint16_t *Data, uint16_t Length)
{
  USB->EPR[EPn] &= 0x870f;   //reset flag CTR_TX
  EpData[EPn].lTX = Length;
  EpData[EPn].pTX_BUFF = Data;
     
  if (EpData[EPn].SendState == EP_SEND_READY)
  {
    if (Length > 0)
      USBLIB_EPBuf2Pma(EPn);
    else
      BDTable[EPn].TX_Count = 0;
  #ifdef SWO_USB_LOG
    pFloat = stradd (pFloat, "\r\nStart TX ");
    pFloat = itoa(Length ,pFloat,10,0);
    pFloat = stradd (pFloat, " b ");
  #endif
    USBLIB_setStatTx(EPn, TX_VALID);
    EpData[EPn].SendState = EP_SEND_BUSY;
  } else
    {
    #ifdef SWO_USB_LOG
      pFloat = stradd (pFloat, "\r\nWait for start TX on EP[");
      pFloat = itoa(EPn ,pFloat,10,0);
      pFloat = stradd (pFloat, "]");
    #endif
    }
}

/* ****************************************************************************
This routine moving data PMA -> User buffer EpData
input: EPn - nomber endpoint buffer
use Global variable EpData, BDTable
**************************************************************************** */
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
  #ifdef SWO_USB_LOG    
  pFloat = stradd (pFloat, "\r\nPMA[");
  pFloat = itoa(EPn ,pFloat,10,0);
  pFloat = stradd (pFloat, "]->Buffer ");
  pFloat = itoa(Count ,pFloat,10,0);
  pFloat = stradd (pFloat, " b: ");
  pFloat = printHexMem(EpData[EPn].pRX_BUFF,pFloat, Count);
#endif
}

/* ****************************************************************************
This routine moving data from user buffer to PMA USB buffer
input: EPn - nomber endpoint buffer
use Global variable EpData, BDTable
**************************************************************************** */
void USBLIB_EPBuf2Pma(uint8_t EPn)
{
  uint32_t *Distination;
  uint8_t   Count;
  Count  = EpData[EPn].lTX <= EpData[EPn].TX_Max ? EpData[EPn].lTX : EpData[EPn].TX_Max;
  BDTable[EPn].TX_Count = Count;
  Distination = (uint32_t *)(USB_PMAADDR + BDTable[EPn].TX_Address * 2);
  #ifdef SWO_USB_LOG
  pFloat = stradd (pFloat, "\r\nBuffer->PMA[");
  pFloat = itoa(EPn ,pFloat,10,0);
  pFloat = stradd (pFloat, "] ");
  pFloat = itoa(Count ,pFloat,10,0);
  pFloat = stradd (pFloat, " b: ");
  pFloat = printHexMem(EpData[EPn].pTX_BUFF,pFloat, Count);
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
Pay attention this bits for can't been directly writen, but only togle 
**************************************************************************** */
void USBLIB_setStatTx(uint8_t EPn, uint16_t Stat)
{
  register uint16_t val = USB->EPR[EPn];
  USB->EPR[EPn] = (val ^ (Stat & EP_STAT_TX)) & (EP_MASK | EP_STAT_TX);
}

/* ****************************************************************************
This routine togled EP_STAT_RX bits. 
Pay attention this bits for can't been directly writen, but only togle 
**************************************************************************** */
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
return value: 0 - if data deny to send. 1-Data accept and will be send
**************************************************************************** */
uint8_t USB_sendReport(uint8_t EPn, uint16_t *Data, uint16_t Length)
{ 
  if ((EpData[EPn].SendState == EP_SEND_READY) && USB_getDeviceConfig() )
  {
    EpData[EPn].lTX = Length;
    EpData[EPn].pTX_BUFF = Data;
    EpData[EPn].SendState = EP_SEND_INITIATE;
    return(1);
  }
  return(0);
}
/* ****************************************************************************
This routine will set USB Address whin next transaction occured
input: address 
**************************************************************************** */
 void     USB_setAddress(uint8_t address)
 {
    DeviceAddress = address;
 }

/* ****************************************************************************
This routine return Endpoint status 
EPnum - endpoint number
return value: 1 - Endpoint HALTED. 0-Endpoint NOT HALTED
**************************************************************************** */
uint16_t  USB_geStatusEP(uint8_t EPnum)
{
  return (uint16_t)(((USB->EPR[EPnum] & RX_VALID) == RX_STALL) ? 1:0);
}


#ifdef SWO_USB_LOG
/* ****************************************************************************
SWO logging for SETUP packet type to SWO by ITM_SendChar
pSetup - pointer on struct
**************************************************************************** */
char *loggingSetupPacket(USB_SetupPacket *pSetup, char *pdst)
{
  pdst = stradd (pdst, "\r\nbmRequestType=");
  pdst = itoa(pSetup->bmRequestType ,pdst,2,0);
  pdst = stradd (pdst, "\r\nbRequest=");
  pdst = itoa(pSetup->bRequest ,pdst,10,0);
  pdst = stradd (pdst, "\r\nwValue.H= ");
  pdst = itoa(pSetup->wValue.H ,pdst,10,0);
  pdst = stradd (pdst, "\r\nwValue.L= ");
  pdst = itoa(pSetup->wValue.L ,pdst,10,0);
  pdst = stradd (pdst, "\r\nwIndex= ");
  pdst = itoa(pSetup->wIndex.L,pdst,10,0);
  pdst = stradd (pdst, "\r\nwLength= ");
  pdst = itoa(pSetup->wLength ,pdst,10,0);
  return(pdst);
}
#endif
