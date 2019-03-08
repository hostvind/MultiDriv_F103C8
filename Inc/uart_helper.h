#ifndef __UART_HELPER_H
#define __UART_HELPER_H
#endif

#include "stm32f1xx_hal.h"
#define IS_DIGIT(x) (((x)>47) && ((x)<58))
/*this grabbed from stdint.h*/

/* exact-width signed integer types */
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

    /* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;



uint8_t atoi8 (uint8_t* start);
uint8_t Parse_Packet (uint8_t *in);
HAL_StatusTypeDef uart_out (char * str);
