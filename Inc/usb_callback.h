#ifndef _USBCALLBACK_H_
 #define _USBCALLBACK_H_

typedef struct 
{
  void (*GetInputReport)    (uint8_t reportNum, uint16_t **ppReport, uint16_t *len);
  void (*GetOutputReport)   (uint8_t reportNum, uint16_t **ppReport, uint16_t *len);
  void (*GetFeatureReport)  (uint8_t reportNum, uint16_t **ppReport, uint16_t *len);
  void (*SetInputReport)    (uint8_t reportNum, uint8_t *pReport, uint16_t len);
  void (*SetOutputReport)   (uint8_t reportNum, uint8_t *pReport, uint16_t len);
  void (*SetFeatureReport)  (uint8_t reportNum, uint8_t *pReport, uint16_t len);
  void (*SetIdle)           (uint8_t *pIdle);
  void (*EPtransmitDone)    (uint8_t EPnum);
  
}Typedef_USB_Callback;

#endif //_USBCALLBACK_H_