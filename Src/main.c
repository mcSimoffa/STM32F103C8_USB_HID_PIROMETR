//SYSCLK=48MHz
#include "config.h"
#include "stm32f103xb_usbdef.h"
#include "cll_stm32F10x_gpio.h"
#include "Systick.h"
#include "additional_func.h"
#include "oringbuf.h"
#include "usb_callback.h"
#include "usb_interface.h"

//for SWO logging activate SWOLOG in Options\C++ compiler\Defined Symbol

#define SYSTICK_DIVIDER 72000
#define PCLK1 36  //APB1 clock (see APB1_DIVIDER in system_stm32f10x.c)
#define MLX90614_ADDR 0x5A
//GPIO A
#define USB_ENABLE 7
#define USB_DM 11
#define USB_DP 12

// GPIO B
#define SCL1  6
#define SDA1  7

// GPIO C
#define ONBOARD_LED 13

//Systick Callback
void FlashOneSec();
void Sender();

//USB Callback
extern Typedef_USB_Callback USB_Callback;
void USB_GetFeature(uint8_t reportN, uint16_t **ppReport, uint16_t *len);
void USB_SetFeature(uint8_t reportN, uint8_t *pReport, uint16_t len);
void USB_SampleSend(uint8_t epn);

struct  //it described in section "feature reports" HID Report Descriptor
  {
    uint16_t  minimum_report_interval;
    uint16_t  report_interval;
    int16_t  maximum_temp;
    int16_t  minimum_temp;
  } HID_SenrorFeature={
    .minimum_report_interval = USB_EP_MIN_REPORT_INTERVAL,
    .report_interval = USB_EP_MIN_REPORT_INTERVAL,
    .maximum_temp = 15000,    //150 degreed Celsium
    .minimum_temp = -4000 };  //-40 degreed Celsium

struct  //it described in section "input reports (transmit)" HID Report Descriptor
{
  uint8_t state;
  uint8_t event;
  int16_t temperature;
} HID_SensorInReport={
  .state = 1,
  .event = 3,
  .temperature = 0 };

uint8_t  isNewSampleTemperature = 1; //flag "Have a new sample"
  
void main()
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN //GPIO C enable  (use as Led)
    | RCC_APB2ENR_IOPAEN    //GPIO A  (use as USB)
    | RCC_APB2ENR_IOPBEN    //GPIO B  (use as I2C) 
    | RCC_APB2ENR_AFIOEN; // Alternate function enable
  //GPIO A init
  PinParametr GPIO_descript[3];
  GPIO_descript[0].PinPos=USB_DM;
  GPIO_descript[0].PinMode=GPIO_MODE_OUTPUT50_ALT_PUSH_PULL;
  
  GPIO_descript[1].PinPos=USB_DP;
  GPIO_descript[1].PinMode=GPIO_MODE_OUTPUT50_ALT_PUSH_PULL;
  
  GPIO_descript[2].PinPos=USB_ENABLE;
  GPIO_descript[2].PinMode=GPIO_MODE_OUTPUT50_OPEN_DRAIN;
  CLL_GPIO_SetPinMode(GPIOA,GPIO_descript,3);   
  GPIO_SET(GPIOA,1<<USB_ENABLE); //Disable USB pullup resistor 1k5
  
  //GPIO B init
  GPIO_descript[0].PinPos=SCL1;
  GPIO_descript[0].PinMode=GPIO_MODE_OUTPUT2_ALT_OPEN_DRAIN;
  
  GPIO_descript[1].PinPos=SDA1;
  GPIO_descript[1].PinMode=GPIO_MODE_OUTPUT2_ALT_OPEN_DRAIN;
  CLL_GPIO_SetPinMode(GPIOB,GPIO_descript,2);   
  
  //GPIO C init
  GPIO_descript[0].PinPos=ONBOARD_LED;
  GPIO_descript[0].PinMode=GPIO_MODE_OUTPUT50_PUSH_PULL;  
  CLL_GPIO_SetPinMode(GPIOC,GPIO_descript,1);
  GPIO_SET(GPIOC,1<<ONBOARD_LED); //off LED 
  
  //I2C1 init
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  I2C1->OAR2  &=  ~I2C_OAR2_ENDUAL;   //Dual adressing disable
  I2C1->CR1 = I2C_CR1_SMBUS;  //SMBus, Device, disable PEC, stretch, Disabled I2C1 peripherial  
  
  I2C1->CR2 = PCLK1;  //Sm mode, 
  
  I2C1->TRISE &=  ~I2C_TRISE_TRISE;
  I2C1->TRISE |=  PCLK1 + 1;  //for 100kHz
  //I2Cx->TRISE |=  PCLK1*300/1000 + 1; //for 400kHz
  I2C1->CCR = 180;  //36 000 000 / (100 000*2)
  
  I2C1->CR1  |= I2C_CR1_PE;
  
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C1->SR1 & I2C_SR1_SB));         // wait SB
  {};
  I2C1->DR = MLX90614_ADDR;
  while (!(I2C1->SR1 & I2C_SR1_ADDR))        // wait ADDR
    {
        if(I2C1->SR1 & I2C_SR1_AF)           // if NACK
            asm("nop");
    }
    (void) I2C1->SR2;                           // clear ADDR
  I2C1->DR=0xF0;//read flag MLX
  
  
#ifdef SWOLOG
   //SWO debug ON
  DBGMCU->CR &= ~(DBGMCU_CR_TRACE_MODE_0 | DBGMCU_CR_TRACE_MODE_0);
  DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;
  // JTAG-DP Disabled and SW-DP Enabled
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;
  
  if (!Oringbuf_Create(10240))
    exceptionFail();   //don't have memory maybe
  uint8_t toSWO;
#endif
  
  
  if (!SysTick_TimersCreate(2))
    exceptionFail();   //don't have memory maybe
  SysTick_TimerInit(0, Sender);
  SysTick_TimerInit(1, FlashOneSec);  //it's no need
  
  //1ms SysTick interval.
  while (SysTick_Config(SYSTICK_DIVIDER)==1)	
    asm("nop");	 //reason - bad divider 
  NVIC_EnableIRQ(SysTick_IRQn);
  SysTick_TimerRun(0, USB_EP_MIN_REPORT_INTERVAL);  //timer to send USB IN Report
  SysTick_TimerRun(1, 1000);  //it's no need
  
#ifdef DEBUG
  //Turn ON the takt core couner
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;// Enable DWT
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; //counter ON
  DWT->CYCCNT = 0; //start value = 0
#endif
  
  //USB callBack function assign
  USB_Callback.GetFeatureReport = USB_GetFeature;
  USB_Callback.SetFeatureReport = USB_SetFeature;
  USB_Callback.EPtransmitDone = USB_SampleSend;
  
  //USB initialize
  GPIO_RESET(GPIOA,1<<USB_ENABLE); //Enable USB pullup resistor 1k5
  //RCC->CFGR |= RCC_CFGR_USBPRE; //not divide PLL clock for USB 48MHz
  RCC->APB1ENR |= RCC_APB1ENR_USBEN; //clocking USB Enable
  USB->CNTR &= ~USB_CNTR_PDWN; //disable PowerDown
  msDelay(1);
  USB->ISTR =0;     //reset all pending interrupts
  USB->BTABLE = 0;  
  USB->CNTR |= USB_CNTR_RESETM;  //USB reset interrupt enable  
  USB->CNTR &= ~USB_CNTR_FRES; //disable Force Reset
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  NVIC_EnableIRQ(USBWakeUp_IRQn);
  //msDelay(4000);
  //GPIO_SET(GPIOA,1<<USB_ENABLE); //Disable USB pullup resistor 1k5
  
  //main cycle
  while (1)
  {
  if(Oringbuf_Get(&toSWO,1))
    ITM_SendChar((uint32_t) toSWO);
  }
}

void FlashOneSec()
{
  asm("nop");
}
      
/* ****************************************************************************
This callback run as the independed timer triggered
it's needs for send IN USB report if temperature refreshed
**************************************************************************** */
void Sender()
{
  HID_SensorInReport.temperature++; //emulated refresh temperaure sensor
  isNewSampleTemperature = 1;
  (GPIO_READ_OUTPUT(GPIOC,1<<ONBOARD_LED)==0) ?  GPIO_SET(GPIOC,1<<ONBOARD_LED):GPIO_RESET(GPIOC,1<<ONBOARD_LED);
  if (USB_sendReport(1,(uint16_t*)&HID_SensorInReport,sizeof(HID_SensorInReport)))
    isNewSampleTemperature = 0;
}

/* ****************************************************************************
This callback run as the GET_REPORT type get feature incoming 
ppReport - pointer to pointer on variable contained data to send
len - pointer to variable contained asked length
application only must to set this two pointers
**************************************************************************** */
void USB_GetFeature(uint8_t reportN, uint16_t **ppReport, uint16_t *len)
{  
  *ppReport = (uint16_t *)&HID_SenrorFeature;
  *len = (*len > sizeof(HID_SenrorFeature)) ? sizeof(HID_SenrorFeature): *len ;
  asm("nop");
}

/* ****************************************************************************
This callback run as the SET_REPORT type get feature incoming
reportN - report number
pReport - pointer to feature Data
len - size of Data
application must Save Data located pReport in owns structure or variable
**************************************************************************** */
void USB_SetFeature(uint8_t reportN, uint8_t *pReport, uint16_t len)
{
  memcpy(&HID_SenrorFeature, pReport, len);
  //change report interval
  SysTick_TimerStop(0);
  SysTick_TimerRun(0, (uint32_t)HID_SenrorFeature.report_interval);
}

/* ****************************************************************************
This callback run after succesfull transmit by Endpoint 
epn - guilty endpoint number
it's need if data sampling faster than Endpoint interrupt period
**************************************************************************** */
void USB_SampleSend(uint8_t epn)
{
  if (epn == 1)
    if (isNewSampleTemperature)
      if (USB_sendReport(epn,(uint16_t*)&HID_SensorInReport,sizeof(HID_SensorInReport)))
        isNewSampleTemperature = 0;
  
}
