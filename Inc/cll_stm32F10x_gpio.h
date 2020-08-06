#ifndef _CLL_STM32F10x_GPIO_H_
#define _CLL_STM32F10x_GPIO_H_
#include "stm32f10x.h"

#define GPIO_Pin_0                 ((uint16_t)0x0001)  /*!< Pin 0 selected */
#define GPIO_Pin_1                 ((uint16_t)0x0002)  /*!< Pin 1 selected */
#define GPIO_Pin_2                 ((uint16_t)0x0004)  /*!< Pin 2 selected */
#define GPIO_Pin_3                 ((uint16_t)0x0008)  /*!< Pin 3 selected */
#define GPIO_Pin_4                 ((uint16_t)0x0010)  /*!< Pin 4 selected */
#define GPIO_Pin_5                 ((uint16_t)0x0020)  /*!< Pin 5 selected */
#define GPIO_Pin_6                 ((uint16_t)0x0040)  /*!< Pin 6 selected */
#define GPIO_Pin_7                 ((uint16_t)0x0080)  /*!< Pin 7 selected */
#define GPIO_Pin_8                 ((uint16_t)0x0100)  /*!< Pin 8 selected */
#define GPIO_Pin_9                 ((uint16_t)0x0200)  /*!< Pin 9 selected */
#define GPIO_Pin_10                ((uint16_t)0x0400)  /*!< Pin 10 selected */
#define GPIO_Pin_11                ((uint16_t)0x0800)  /*!< Pin 11 selected */
#define GPIO_Pin_12                ((uint16_t)0x1000)  /*!< Pin 12 selected */
#define GPIO_Pin_13                ((uint16_t)0x2000)  /*!< Pin 13 selected */
#define GPIO_Pin_14                ((uint16_t)0x4000)  /*!< Pin 14 selected */
#define GPIO_Pin_15                ((uint16_t)0x8000)  /*!< Pin 15 selected */
#define GPIO_Pin_All               ((uint16_t)0xFFFF)  /*!< All pins selected */

//Function for atomic SET&RESET, Reset, Set, Read Input, Read Output
#define GPIO_SET_RESET(PORT,GPIO_PIN_MASK_SET,GPIO_PIN_MASK_RESET) (PORT->BSRR = (uint32_t)(GPIO_PIN_MASK_SET) | (uint32_t)(GPIO_PIN_MASK_RESET)<<16)
#define GPIO_RESET(PORT,GPIO_PIN_MASK_RESET) (PORT->BRR = (uint32_t)(GPIO_PIN_MASK_RESET))
#define GPIO_SET(PORT,GPIO_PIN_MASK_SET) (PORT->BSRR = (uint32_t)(GPIO_PIN_MASK_SET))
#define GPIO_READ_INPUT(PORT,GPIO_PIN_MASK) (uint16_t)(((uint16_t)PORT->IDR) & (GPIO_PIN_MASK))
#define GPIO_READ_OUTPUT(PORT,GPIO_PIN_MASK) (uint16_t)(((uint16_t)PORT->ODR) & (GPIO_PIN_MASK))



typedef enum
{
GPIO_MODE_INPUT_ANALOG               =          0x00,
GPIO_MODE_INPUT_FLOATING             =          0x04,
GPIO_MODE_INPUT_PULL_UP_DOWN         =          0x08,
GPIO_MODE_OUTPUT2_PUSH_PULL          =          0x02,
GPIO_MODE_OUTPUT2_OPEN_DRAIN         =          0x06,
GPIO_MODE_OUTPUT2_ALT_PUSH_PULL      =          0x0A,
GPIO_MODE_OUTPUT2_ALT_OPEN_DRAIN     =          0x0E,
GPIO_MODE_OUTPUT10_PUSH_PULL         =          0x01,
GPIO_MODE_OUTPUT10_OPEN_DRAIN        =          0x05,
GPIO_MODE_OUTPUT10_ALT_PUSH_PULL     =          0x09,
GPIO_MODE_OUTPUT10_ALT_OPEN_DRAIN    =          0x0D,
GPIO_MODE_OUTPUT50_PUSH_PULL         =          0x03,
GPIO_MODE_OUTPUT50_OPEN_DRAIN        =          0x07,
GPIO_MODE_OUTPUT50_ALT_PUSH_PULL     =          0x0B,
GPIO_MODE_OUTPUT50_ALT_OPEN_DRAIN    =          0x0F,
GPIO_MODE_RESERVED                   =          0x0C
} GPIOMode_TypeDef;

typedef struct 
{
char PinPos;
GPIOMode_TypeDef PinMode; 
} PinParametr;


void CLL_GPIO_SetPinMode(GPIO_TypeDef *GPIOx, const PinParametr *Mode, const char ElemCount);
void CLL_GPIO_SetOnePinMode(GPIO_TypeDef *GPIOx,  const uint8_t Pin_Num, GPIOMode_TypeDef PinParametr);
#endif //_CLL_STM32F10x_GPIO_H_