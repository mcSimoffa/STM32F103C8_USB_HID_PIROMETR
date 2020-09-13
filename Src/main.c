//SYSCLK=48MHz
#include "config.h"
#include "cll_stm32F10x_gpio.h"
#include "Systick.h"
#include "additional_func.h"
#include "oringbuf.h"
#include "stm32f103xb_usbdef.h"
#include "usb_callback.h"
#include "usb_interface.h"
#include "i2c.h"

//for USB SWO logging activate SWO_USB_LOG in Options\C++ compiler\Defined Symbol
//for I2C SWO logging activate SWO_I2C_LOG in Options\C++ compiler\Defined Symbol

#define SYSTICK_DIVIDER 72000
#define MLX90614_ADDR 0x5A    //standard Slave Address for MLX90614
#define DELTA_KELVIN_CELSIUM  27315 // 0K = 273.15C
#define PCLK1 36  //APB1 clock (see APB1_DIVIDER in system_stm32f10x.c)

//Sensor state: extract from report descriptor
#define SENSOR_STATE_UNKNOWN        0x00
#define SENSOR_STATE_READY          0x01
#define SENSOR_STATE_NOT_AVAILABLE  0x02
#define SENSOR_STATE_NO_DATA        0x03
#define SENSOR_STATE_INITIALIZING   0x04
#define SENSOR_STATE_ACCESS_DENIED  0x05
#define SENSOR_STATE_ERROR          0x06

//Sensor event: extract from report descriptor
#define SENSOR_EVENT_UNKNOWN        0x00
#define SENSOR_EVENT_STATE_CHANGED  0x01
#define SENSOR_EVENT_DATA_UPDATED   0x03

//GPIO A
#define USB_ENABLE 7
#define USB_DM 11
#define USB_DP 12

// GPIO B
#define SCL1  6
#define SDA1  7

// GPIO C
#define ONBOARD_LED 13

//I2C callback
void GotMLXtemperature(uint16_t *pData);

//Systick Callback
void FlashOneSec();
void Sender();

//USB Callback 
extern Typedef_USB_Callback USB_Callback;
void USB_GetFeature(uint8_t reportN, uint16_t **ppReport, uint16_t *len);
void USB_SetFeature(uint8_t reportN, uint8_t *pReport, uint16_t len);
void USB_SetIdle(uint8_t *idle);

struct  //it described in section "feature reports" HID Report Descriptor
  {
    uint16_t  minimum_report_interval;
    uint16_t  report_interval;
    int16_t  maximum_temp;
    int16_t  minimum_temp;
  } HID_SenrorFeature={
    .minimum_report_interval = USB_EP_MIN_REPORT_INTERVAL,
    .report_interval = REPORT_INTERVAL_AT_START,
    .maximum_temp = 15000,    //150 degreed Celsium
    .minimum_temp = -4000 };  //-40 degreed Celsium

struct  //it described in section "input reports (transmit)" HID Report Descriptor
{
  uint8_t state;
  uint8_t event;
  int16_t temperature;
} HID_SensorInReport={
  .state = SENSOR_STATE_UNKNOWN,
  .event = SENSOR_EVENT_UNKNOWN,
  .temperature = 0 };

uint8_t  isNewSampleTemperature = 0; //flag "Have a new sample"
  
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
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; //Clock I2C must be enabled BEFORE be GPIO init which links on I2C !!!!!!!! Else BUSY flag permanent ON
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
  
 #if defined (SWO_USB_LOG) || (SWO_SMBUS_LOG)
   //SWO debug ON
  DBGMCU->CR &= ~(DBGMCU_CR_TRACE_MODE_0 | DBGMCU_CR_TRACE_MODE_0);
  DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;
  // JTAG-DP Disabled and SW-DP Enabled
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;
  
  //Turn ON the takt core couner
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;// Enable DWT
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; //counter ON
  DWT->CYCCNT = 0; //start value = 0
  
  if (!Oringbuf_Create(10240))
    exceptionFail();   //don't have memory maybe
  uint8_t toSWO;
#endif
  
  //I2C1 init
  I2C1->OAR2  &=  ~I2C_OAR2_ENDUAL;   //Dual adressing disable
  I2C1->CR1 = I2C_CR1_SMBUS;  //SMBus, Device, disable PEC, stretch, Disabled I2C1 peripherial    
  I2C1->CR2 = PCLK1
            | I2C_CR2_ITBUFEN
            | I2C_CR2_ITEVTEN
            | I2C_CR2_ITERREN;  //Sm mode, Interrupt EVENT & ERROR enabled
  
  I2C1->TRISE &=  ~I2C_TRISE_TRISE;
  I2C1->TRISE |=  PCLK1 + 1;  //for 100kHz
  I2C1->CCR = 180;  //36 000 000 / (100 000*2)
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_EnableIRQ(I2C1_ER_IRQn);
  I2C1->CR1  |= I2C_CR1_PE; //Enable I2C1
 
  if (!SysTick_TimersCreate(2))
    exceptionFail();   //don't have memory maybe
  SysTick_TimerInit(0, Sender);
  SysTick_TimerInit(1, FlashOneSec);  //it's no need
  
  //1ms SysTick interval.
  while (SysTick_Config(SYSTICK_DIVIDER)==1)	
    asm("nop");	 //reason - bad divider 
  NVIC_EnableIRQ(SysTick_IRQn);
  SysTick_TimerRun(1, 1000);  //it's no need
  
  //USB callBack function assign
  USB_Callback.GetFeatureReport = USB_GetFeature;
  USB_Callback.SetFeatureReport = USB_SetFeature;
  USB_Callback.SetIdle = USB_SetIdle;

  
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
#if defined (SWO_USB_LOG) || (SWO_SMBUS_LOG)
  if(Oringbuf_Get(&toSWO,1))
    ITM_SendChar((uint32_t) toSWO);
#endif  
  }
}

void FlashOneSec()
{
  asm("nop");
}

/* ****************************************************************************
This callback called when the SET_IDLE request incoming
and it Turn ON timer for sampling the temperature Sensor
idle - pointer to IDLE value. Usually it's Zero
**************************************************************************** */
void USB_SetIdle(uint8_t *idle)
{
  SysTick_TimerRun(0, REPORT_INTERVAL_AT_START);  //sampling temperature rate from MLX90614
}
/* ****************************************************************************
This callback called as the independed timer triggered
it's launch data transmit from MLX90614 to SMT32
**************************************************************************** */
void Sender()
{
  //here convenient to use Break Pint for  report send step By step
  if (isNewSampleTemperature)
    if (USB_sendReport(1,(uint16_t*)&HID_SensorInReport,sizeof(HID_SensorInReport)))
      isNewSampleTemperature = 0;
  
   if (!I2C_ReadWord(MLX90614_ADDR, 0x07, GotMLXtemperature))
    {
      HID_SensorInReport.state = SENSOR_STATE_ERROR;  //failed launch transrer temperature data from MLX90614
      HID_SensorInReport.event = SENSOR_EVENT_STATE_CHANGED;
      isNewSampleTemperature = 1;
    }
}

/* ****************************************************************************
This callback called as the pirometr semsor MLX90614 take the temperature value
pData - pointer to word of temperature value
**************************************************************************** */
void GotMLXtemperature(uint16_t *pData)
{
  static uint16_t MLXpreviousData;
  if (MLXpreviousData != *pData)
  {
    (GPIO_READ_OUTPUT(GPIOC,1<<ONBOARD_LED)==0) ?  GPIO_SET(GPIOC,1<<ONBOARD_LED):GPIO_RESET(GPIOC,1<<ONBOARD_LED); //LED indication
    isNewSampleTemperature = 1;
    MLXpreviousData = *pData;
    HID_SensorInReport.temperature = (MLXpreviousData << 1) - DELTA_KELVIN_CELSIUM; //convertion to Celsium degreeds 
    HID_SensorInReport.state = SENSOR_STATE_READY;
    HID_SensorInReport.event = SENSOR_EVENT_DATA_UPDATED;
  }
}

/* ****************************************************************************
This callback called as the GET_REPORT type get feature incoming 
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
This callback called as the SET_REPORT type get feature incoming
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

