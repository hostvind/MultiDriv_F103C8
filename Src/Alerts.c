#include "Alerts.h"

    LED_driver LD1, LD2, LD3, LD4;
    LED_driver* LED_p[4];
    BUZ_driver Buzzer;



void BUZZ (uint8_t time_x100ms)
    {
        if (time_x100ms < 101)
        {
            Buzzer.timer = time_x100ms+1;  //due to pre-decrement in ISR it stops on reaching 1
            Buzzer.State = START;
            TIM4->DIER |= TIM_IT_CC1;
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
