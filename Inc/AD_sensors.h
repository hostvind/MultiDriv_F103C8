#include "stm32f1xx_hal.h"


/*  CONSIDER INTEGRATING IN LV3.H           */
/*  MUST BE INITIALIZED IN main.c main()    */
/*  FOR PINBOARD II BY DEFAULT IS           */
//    LD1.LED_Port = GPIOA;   LD1.LED_Pin = GPIO_PIN_15;
//    LD2.LED_Port = GPIOB;   LD2.LED_Pin = GPIO_PIN_3;
//    LD3.LED_Port = GPIOB;   LD3.LED_Pin = GPIO_PIN_4;
//    LD4.LED_Port = GPIOB;   LD4.LED_Pin = GPIO_PIN_5;
//    LED_p[0]=&LD1; LED_p[1]=&LD2; LED_p[2]=&LD3; LED_p[3]=&LD4; 
//    
//    Buzzer.TIM_Handle = &htim2;
//    Buzzer.TIM_Channel = TIM_CHANNEL_1;
/*  COPY AND PASTE IF NO CHANGES PENDING*/

typedef enum {
    STATE_OFF,
    STATE_ON,
} SN_STATE;
typedef enum {
    SN_HYG,
    SN_RIN,
    SN_GAS
} SN_TYPE;
typedef struct {
    GPIO_TypeDef* SENS_PortA;
    uint16_t SENS_PinA;
    ADC_HandleTypeDef* hadc;
    GPIO_TypeDef* SENS_PortD;
    uint16_t SENS_PinD;
    volatile uint32_t* val;
    volatile SN_STATE State;
    SN_TYPE Type;
} SENS_driver;
//typedef struct {
//    GPIO_TypeDef* GAS_PortA;
//    uint16_t HYD_PinA;
//    GPIO_TypeDef* GAS_PortD;
//    uint16_t GAS_PinD;
//    volatile uint16_t val;
//    volatile STATE State;
//} GAS_driver;
//typedef struct {
//    GPIO_TypeDef* RN_PortA;
//    uint16_t HYD_PinA;
//    GPIO_TypeDef* RN_PortD;
//    uint16_t HYD_PinD;
//    volatile uint16_t val;
//    volatile STATE State;
//} RN_driver;

uint16_t Sensor_Get_Data (SENS_driver* sensor);
void Sensor_Alert (SENS_driver* sensor);
