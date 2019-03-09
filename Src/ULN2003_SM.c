#include "ULN2003_SM.h"
//ULN2003_StepMotor
uint16_t CCW[8] = {0x9000,0x1000,0x3000,0x2000,0x6000,0x4000,0xc000,0x8000};
uint16_t  CW[8] = {0x8000,0xc000,0x4000,0x6000,0x2000,0x3000,0x1000,0x9000};

void ULN2003_go (SM_driver* SM, uint16_t time, uint8_t period, uint8_t direction)
{
	SM->step = 0;
    if (SM->direction)  SM->coils = CCW;
    else                SM->coils = CW;
    SM->time = time;
    SM->period = period;
    SM->htim->Instance->ARR=SM->period;
    SM->htim->Instance->CCR1=SM->period;
    HAL_TIM_OC_Start_IT (SM->htim, TIM_CHANNEL_1);
//	while (step < 8)
//	{	        
//        if (direction) SM->SM_Port->ODR = (SM->SM_Port->ODR&0x0FFF)|CCW[step];
//        else SM->SM_Port->ODR = (SM->SM_Port->ODR&0x0FFF)|CW[step];
//		step += 1;
//		HAL_Delay(time);
//	}
//    SM->SM_Port->ODR = (SM->SM_Port->ODR&0x0FFF);
    return;
}

void ULN2003_stop (SM_driver* SM)
{
    HAL_TIM_OC_Stop_IT (SM->htim, TIM_CHANNEL_1);
    return;
}
