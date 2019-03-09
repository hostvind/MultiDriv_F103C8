#include "AD_sensors.h"

uint16_t Sensor_Get_Data (SENS_driver* sensor)
{
    uint16_t data;
    
    data = (uint16_t) *sensor->val;
    if (HAL_GPIO_ReadPin (sensor->SENS_PortD, sensor->SENS_PinD))
        sensor->State=STATE_OFF;
    else sensor->State=STATE_ON;
    if (sensor->Type == SN_GAS)
    {
        if (data>2000)
            data=2000;   //TO DO
        data=data/20;
    }
    else
    {
        if (data>4000)
            data=4000;
        if (data<1000)
            data=1000;
        data=(100-(data-1000)/30); //resistance 0-100% humidity = 100-resistance
//      data=100-data; 
    }
    return data;
}
void Sensor_Alert (SENS_driver* sensor)
{
    ;
}
