#include "uart_helper.h"
#include "string.h"
#include "Alerts.h"

extern UART_HandleTypeDef huart1;
volatile uint8_t uart_in[64];
volatile unsigned char * uart_in_p = (unsigned char *) &uart_in;

uint8_t uart_str[64] = "";
unsigned char* uart_str_p = (unsigned char *)uart_str;

HAL_StatusTypeDef uart_out (char * str)
{
    if (strlen (str) < 65)
        sprintf(uart_str_p, str);
    else
        strncpy (uart_str_p, str, 64);
    return HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str_p), 100);
}

uint8_t atoi8 (uint8_t* start)
{
    uint8_t     i=0;
    uint16_t    res=0;  //for not to crop on "res*=10"
    for (i=0;i<32;i++)  //for if there's no digits at all don't run away too far
        while (!IS_DIGIT(*start))
            start++;
    for (i=0;i<3;i++)   //since it's 0..255 read no more than 3 chars
        {
            if (IS_DIGIT(start[i]))
            {
                res+=start[i]-48;
                res*=10;
            }
            else break;
        }
        return (uint8_t) (res/10);
}


uint8_t Parse_Packet (uint8_t *in)
{
	
	uint8_t *p;
	uint8_t temp;
	uint8_t led;
    if (!strncmp ((char *)in,"BUZZ",4))
    {
        if (in[4]=='\f' || in[4]=='\n')
            BUZZ(2);
        if (in[4]==' ')
            BUZZ(atoi8(&in[5]));
        return 0;
    }
    if (!strncmp ((char *)in,"BLINK",5))
    {
        if (in[5]=='\f' || in[5]=='\n')
            BLINK(3,4,ONCE);
        if (in[5]==' ')
        {
            //parse input values
            p = &in[6];
            temp=atoi8(p); //GET TIME
            led=0;        //GET LED
            while (IS_DIGIT(*p)) p++;
            p++;
            led = (*p) - 48;
            if (!( (led>0) && (led<5) ))    //wrong LED selected - do nothing
                return 2;
            p+=2;
            if (!strncmp ((char *)p,"PERM",4))
                BLINK (temp, led, PERM);
            else BLINK (temp,led, ONCE);     //default must be "ONCE"
        }
        return 0;
    }
    
    else return 1;                      //command not recognized
}
