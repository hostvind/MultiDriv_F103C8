#include "stm32f1xx_hal.h"

 #define CLOCKWISE          0
 #define COUNTERLOCKWISE    1
//presume that all lines are connected to PORTB 12-15
typedef struct {
    GPIO_TypeDef* SM_Port;
    uint8_t step;
    uint8_t direction;    
    uint16_t* coils;
    uint16_t time;
    uint8_t period;
    TIM_HandleTypeDef* htim;
} SM_driver;

void ULN2003_go (SM_driver* SM, uint16_t time, uint8_t period, uint8_t direction);
void ULN2003_stop (SM_driver* SM);
