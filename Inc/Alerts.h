#ifndef __ALERTS_H
#define __ALERTS_H
#endif
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
    STOP,
    RUN,
    START
} STATE;
typedef enum {    
    ONCE,
    PERM
} MODE;

typedef struct {
    GPIO_TypeDef* LED_Port;
    uint16_t LED_Pin;
    uint8_t timer;
    volatile uint8_t counter;
    volatile STATE State;
    MODE Mode;
} LED_driver;
typedef struct {
    TIM_HandleTypeDef* TIM_Handle;
    uint32_t TIM_Channel;
    uint8_t timer;
    volatile uint8_t counter;
    volatile STATE State;
    MODE Mode;    
} BUZ_driver;
void Alarm_On (uint16_t frequency);
void Alarm_Off (void);
void BUZZ (uint8_t time_x100ms);
void BLINK (uint8_t time_x100ms, uint8_t led, MODE mode);
