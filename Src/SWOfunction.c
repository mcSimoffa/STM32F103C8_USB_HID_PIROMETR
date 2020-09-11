#include "SWOfunction.h"
#include "oringbuf.h"
#include "additional_func.h"

void putlog(void *pBegin, void *pEnd)
{
  uint16_t len=(uint8_t*)pEnd - (uint8_t*)pBegin;
  if (len)
    if (Oringbuf_Put(pBegin, len) < len)
      exceptionFail();   //debug buffer is owerflowed
  return;
}

/* *************************************************************
 Send to ITM port knowed lenth string
*pStr - pointer of sended string
len - length of string
************************************************************* */
void ITM_SendString(uint8_t *pStr, int32_t len)
{
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*pStr++));
}

/* *************************************************************
 Send to ITM port null terminated string
*toSWO - pointer of sended string
************************************************************* */
void debugPrint (char *toSWO)
{
 while ( *toSWO!=0)
  ITM_SendChar((uint32_t)*(toSWO++));
 return; 
}
