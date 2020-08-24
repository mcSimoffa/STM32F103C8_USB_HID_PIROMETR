#ifndef _ADD_FUNC_
  #define _ADD_FUNC_
  #include "stm32f10x.h"
  void ITM_SendString(uint8_t *pStr, int32_t len);
  void debugPrint (char *toSWO);
  char *itoa(int32_t sr, char *s, uint8_t radix, uint8_t isSigned);
  char *stradd (char * destptr, char * srcptr );
#endif //_ADD_FUNC_