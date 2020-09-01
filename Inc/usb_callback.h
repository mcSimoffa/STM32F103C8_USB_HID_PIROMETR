#ifndef _USBCALLBACK_H_
 #define _USBCALLBACK_H_

typedef struct 
{
  void (*GetInputReport)   (uint16_t **ppReport, uint16_t *len);
  void (*GetOutputReport)  (uint16_t **ppReport, uint16_t *len);
  void (*GetFeatureReport) (uint16_t **ppReport, uint16_t *len);
  void (*SetInputReport)   (uint8_t *pReport, uint16_t len);
  void (*SetOutputReport)  (uint8_t *pReport, uint16_t len);
  void (*SetFeatureReport) (uint8_t *pReport, uint16_t len);  
  
}Typedef_USB_Callback;

#endif //_USBCALLBACK_H_