#include "oringbuf.h"

uint8_t *head=0;
uint8_t  *tail=0;
uint16_t size=0;
uint8_t *lowlimit, *highlimit;

/* **************************************************************     
Create origin buffer
_size - size of buffer in bytes
return value - pointer of head
or -1 if origin buffer wasn't created
************************************************************** */
void *Oringbuf_Create(uint16_t _size)
{
  
  head = malloc(_size);
  if (head)
    size = _size;
  tail = lowlimit = head;
  highlimit = (uint8_t*)lowlimit + size;
  return (head);
}

/* **************************************************************     
Get a free place in origin buffer
return value - available free size in bytes
************************************************************** */
uint16_t Oringbuf_GetFree()
{
  uint16_t retval;
  uint8_t *s_head=head;
  uint8_t *s_tail=tail;
  if(size>0)
    
    retval = (s_head>=s_tail) ? (size-(uint16_t)(s_head-s_tail)) : (uint16_t)(s_tail-s_head);
    else
      retval = 0;
    if (retval)
      retval--;
  return(retval);  
}

/* **************************************************************     
Get a filled in origin buffer
return value - contain size in bytes
************************************************************** */
uint16_t Oringbuf_GetFilled()
{
  uint8_t *s_head=head;
  uint8_t *s_tail=tail;
  return ((s_head>=s_tail) ? (uint16_t)(s_head-s_tail) : (size-(uint16_t)(s_tail-s_head))) ; 
}

/* **************************************************************     
put the array to head.
psrc - pointer of source byte array
cnt -  source array length
return value - size of the array that was puted
************************************************************** */
uint16_t Oringbuf_Put(void *psrc, uint16_t cnt)
{
  uint16_t vol = Oringbuf_GetFree();
  vol = (cnt < vol) ? cnt : vol;
  uint16_t volfirst = (vol <= (uint16_t)((uint8_t*)highlimit-(uint8_t*)head)) ? vol : (uint16_t)((uint8_t*)highlimit-(uint8_t*)head);
  uint16_t volsecond = vol - volfirst;
  memcpy(head,psrc,volfirst);
  head = (uint8_t*)head + volfirst;
  if (head >= highlimit)
    head = (uint8_t*)head - size;
  if(volsecond)
  {
    memcpy(head,(uint8_t*)psrc+volfirst,volsecond);
    head = (uint8_t*)head + volsecond;
  }
  return(vol);
}

void *Oringbuf_Gethead()
{
  return (head);
}

uint16_t Oringbuf_Get(void *pdst, uint16_t cnt)
{
  uint16_t vol = Oringbuf_GetFilled();
  vol = (cnt < vol) ? cnt : vol;
  uint16_t volfirst = (vol <= (uint16_t)((uint8_t*)highlimit-(uint8_t*)tail)) ? vol : (uint16_t)((uint8_t*)highlimit-(uint8_t*)tail);
  uint16_t volsecond = vol - volfirst;
  memcpy(pdst,tail,volfirst);
  tail = (uint8_t*)tail + volfirst;
  if (tail >= highlimit)
    tail = (uint8_t*)tail - size;
  if(volsecond)
  {
    memcpy((uint8_t*)pdst+volfirst,tail,volsecond);
    tail = (uint8_t*)tail + volsecond;
  }
  return(vol);
}