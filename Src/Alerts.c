#include "Alerts.h"

    LED_driver LD1, LD2, LD3, LD4;
    LED_driver* LED_p[4];
    BUZ_driver Buzzer;

void Alarm_On (uint16_t frequency)
{
//    HAL_TIM_PWM_Stop (Buzzer.TIM_Handle, Buzzer.TIM_Channel);
    TIM2->ARR=SystemCoreClock/frequency;
    TIM2->CCR2=(SystemCoreClock/frequency)/2;
    HAL_TIM_PWM_Start (Buzzer.TIM_Handle, Buzzer.TIM_Channel);
//    TIM2->DIER |= TIM_IT_CC1;
//            switch (Buzzer.TIM_Channel)
//            {                
//                case TIM_CHANNEL_1: TIM2->DIER |= TIM_IT_CC1;
//                Buzz.TIM_Handle->Instance->CCR1=(frequency/440)/2;
//                break;
//                case TIM_CHANNEL_2: TIM2->DIER |= TIM_IT_CC2;
//                Buzz.TIM_Handle->Instance->CCR2=(frequency/440)/2;
//                break;
//                case TIM_CHANNEL_3: TIM2->DIER |= TIM_IT_CC3;
//                Buzz.TIM_Handle->Instance->CCR3=(frequency/440)/2;
//                break;
//                case TIM_CHANNEL_4: TIM2->DIER |= TIM_IT_CC4;
//                Buzz.TIM_Handle->Instance->CCR4=(frequency/440)/2;
//                break;
//                default:  TIM2->DIER |= TIM_IT_CC1;
//                Buzz.TIM_Handle->Instance->CCR1=(frequency/440)/2;
//                break;
//            }
    return;
}

void Alarm_Off (void)
{
    HAL_TIM_PWM_Stop (Buzzer.TIM_Handle, Buzzer.TIM_Channel);
//    TIM2->DIER &= ~(TIM_IT_CC1);
//    switch (Buzzer.TIM_Channel)
//    {                
//        case TIM_CHANNEL_1: TIM2->DIER &= ~(TIM_IT_CC1);
//        break;
//        case TIM_CHANNEL_2: TIM2->DIER &= ~(TIM_IT_CC2);
//        break;
//        case TIM_CHANNEL_3: TIM2->DIER &= ~(TIM_IT_CC3);
//        break;
//        case TIM_CHANNEL_4: TIM2->DIER &= ~(TIM_IT_CC4);
//        break;
//        default:  TIM4->DIER &= ~(TIM_IT_CC1);
//        break;
//    }
    return;
}

void BUZZ (uint8_t time_x100ms)
    {
        Buzzer.TIM_Handle->Instance->ARR=SystemCoreClock/440;
        Buzzer.TIM_Handle->Instance->CCR1=(SystemCoreClock/440)/2;
        if (time_x100ms < 101)
        {
            Buzzer.timer = time_x100ms+1;  //due to pre-decrement in ISR it stops on reaching 1
            Buzzer.State = START;
            switch (Buzzer.TIM_Channel)
            {                
                case TIM_CHANNEL_1: TIM4->DIER |= TIM_IT_CC1;
                break;
                case TIM_CHANNEL_2: TIM4->DIER |= TIM_IT_CC2;
                break;
                case TIM_CHANNEL_3: TIM4->DIER |= TIM_IT_CC3;
                break;
                case TIM_CHANNEL_4: TIM4->DIER |= TIM_IT_CC4;
                break;
                default:  TIM4->DIER |= TIM_IT_CC1;
                break;
            }
        }
        else return;
        return;
    }
    
    /*Send time=0 to stop*/
void BLINK (uint8_t time_x100ms, uint8_t led, MODE mode)
    {
        switch (led)
        {
            case 1: 
                {
                LD1.timer = time_x100ms;
                LD1.Mode  = mode;
                if (!time_x100ms)
                    LD1.State = STOP;
                else    
                LD1.State = START;
                break;
                };
            case 2: 
                {
                LD2.timer = time_x100ms;
                LD2.Mode  = mode;
                if (!time_x100ms)
                    LD2.State = STOP;
                else  
                LD2.State = START;
                break;
                }
            case 3: 
                {
                LD3.timer = time_x100ms;
                LD3.Mode  = mode;
                if (!time_x100ms)
                    LD3.State = STOP;
                else  
                LD3.State = START;
                break;
                }
            case 4: 
                {
                LD4.timer = time_x100ms;
                LD4.Mode  = mode;
                if (!time_x100ms)
                    LD4.State = STOP;
                else  
                LD4.State = START;
                break;
                }
            default:
            {
                return;
            }
        }
        return;
    }
