#ifndef _ORINGBUF_H_
 #define _ORINGBUF_H_
 #include "stm32f10x.h"
 #include  <stdlib.h>
 #include  <string.h>

void *Oringbuf_Create(uint16_t _size);
uint16_t Oringbuf_GetFree();
uint16_t Oringbuf_GetFilled();
uint16_t Oringbuf_Put(void *psrc, uint16_t cnt);
uint16_t Oringbuf_Get(void *pdst, uint16_t cnt);
#endif //_ORINGBUF_H_