#include "additional_func.h"
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

/* **************************************************************     
convert uint32_t to string routine.
sr - source numer
*s - destination string pointer. String should have enough length
radix - radix of system. example 2 or 10 or 16
isSigned - flag. How to use sr: 0 - unsigned value, 1- signed value
return value - pointer on null terminator at end of string
************************************************************** */
char *itoa(int32_t sr, char *s, uint8_t radix, uint8_t isSigned)
{
  char c;   //for reverse
  int32_t sign=1;
  uint32_t n;
  char *pBeginStr=s;
  uint8_t tetracnt=0;
  if (isSigned)
  {
    if ((sign = sr) < 0)
      n = -sr;
  }
  else
    n=(uint32_t)sr;
  
  *(s++)='\0';
  do 
  { 
    if((radix == 2) && (tetracnt++ == 4))
    {
      *(s++) = ' ';
      tetracnt = 1;
    }
    *s = n % radix;
     if (*s > 9)
       *s += ('A'-10);
      else
       *s += '0'; 
    s++;
  } while ((n /= radix) > 0);
  if (sign < 0)
    *s++ = '-';
  if (radix == 2)
    {
      for(uint8_t i=tetracnt;i<4;i++)
        *s++ = '0';
      *s++ = 'B';
    }
  if (radix == 16)
    {
      *s++ = 'x';
      *s++ = '0';
    }
  char *p=s;    //end of string pointer before reverse
  p--;
  char *retVal = --s;
  s=pBeginStr;
  do
  {
    c = *s;
    *s++ = *p;
    *p-- = c;
  } while (p-s>0);
  return (retVal);
}

/* ***********************************************************
It addition null-terminated string from 'srcptr' address
to 'destptr' addres
It return pointer of null terminator of new string
*********************************************************** */
char *stradd (char * destptr, char * srcptr )
{
  do
    *destptr++ = *srcptr;
  while (*(srcptr++)!=0x00);
  return (--destptr);
}