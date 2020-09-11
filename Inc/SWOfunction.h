#ifndef _SWO_FUNC_
  #define _SWO_FUNC_
  #include "stm32f10x.h"
  void putlog(void *pbegin, void *pEnd);
  void ITM_SendString(uint8_t *pStr, int32_t len);
  void debugPrint (char *toSWO);
#endif  //_SWO_FUNC_