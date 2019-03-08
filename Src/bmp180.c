/*
 * bmp180.c
 *
 *  Created on: 11 feb. 2019
 *      Author: gheorghe
 */

#include "bmp180.h"

#define _AC1 bmp->calib_data.AC1
#define _AC2 bmp->calib_data.AC2
#define _AC3 bmp->calib_data.AC3
#define _AC4 bmp->calib_data.AC4
#define _AC5 bmp->calib_data.AC5
#define _AC6 bmp->calib_data.AC6
#define _B1 bmp->calib_data.B1
#define _B2 bmp->calib_data.B2
#define _MB bmp->calib_data.MB
#define _MC bmp->calib_data.MC
#define _MD bmp->calib_data.MD

extern I2C_HandleTypeDef hi2c1;
 bmp_data_t				dataBMP;


 int32_t read_oss(void)
 {
	int32_t oss = 0;
	uint8_t arr[2] = {0};
	BMP_SET_START_ADDR ( arr[0], BMP_CTRL_MEAS_REG );

	HAL_I2C_Master_Transmit ( &hi2c1, BMP_WRITE_ADDR, arr, 1, 200 );
	HAL_I2C_Master_Receive ( &hi2c1, BMP_READ_ADDR, arr, 1, 200 );

	oss = READ_OSS(arr[0]);
	return oss;
 }

 int32_t read_UT(void)
 {
 	uint8_t buff[2] = {0};

 	BMP_SET_START_ADDR ( buff[0], BMP_START_TEMP );
 	//buff[0] = 0x2E
 	HAL_I2C_Mem_Write( &hi2c1, BMP_WRITE_ADDR, BMP_CTRL_MEAS_REG, 1, &buff[0], 1, 200 );
 	//write 0x2E into reg 0xF4 wait 4.5ms

 	HAL_Delay(10);

 	HAL_I2C_Mem_Read ( &hi2c1, BMP_READ_ADDR, BMP_DATA_MSB_ADDR, 1, &buff[0], 1, 200 );
 	HAL_I2C_Mem_Read ( &hi2c1, BMP_READ_ADDR, BMP_DATA_LSB_ADDR, 1, &buff[1], 1, 200 );
 	//read reg 0xF6(MSB), 0xF7(LSB)
 	//buff[0](MSB), buff[1](LSB)

 	return ((buff[0] << 8) + buff[1]);
 }


 int32_t read_UP(void)
 {
 	uint8_t buff[3] = {0};
 	int32_t up = 0;


 	BMP_SET_START_ADDR ( buff[0], BMP_PRESS_OSS_0 + (dataBMP.oss << 6));
 	//buff[0] = 0x34 + (oss << 6)
 	HAL_I2C_Mem_Write( &hi2c1, BMP_WRITE_ADDR, BMP_CTRL_MEAS_REG, 1, &buff[0], 1, 200 );
 	//write (buff[0] =  0x34 + (oss << 6)) into reg 0xF4

 	HAL_Delay(100);

 	HAL_I2C_Mem_Read ( &hi2c1, BMP_READ_ADDR, BMP_DATA_MSB_ADDR, 1, &buff[0], 1, 200 );
 	HAL_I2C_Mem_Read ( &hi2c1, BMP_READ_ADDR, BMP_DATA_LSB_ADDR, 1, &buff[1], 1, 200 );
 	HAL_I2C_Mem_Read ( &hi2c1, BMP_READ_ADDR, BMP_DATA_XLSB_ADDR, 1, &buff[2], 1, 200 );
 	//read reg 0xF6(MSB), 0xF7(LSB), 0xF8(XLSB)
 	//buff[0](MSB), buff[1](LSB), buff[1](XLSB)

 	up = ((buff[0]<<16) + (buff[1]<<8) + buff[2]) >> (8 - dataBMP.oss);
 	//UP = (MSB<<16 + LSB<<8 + XLSB) >> (8 - oss)
 	return up;
 }

void read_calib_data(bmp_calib_param_t *calib_data)
{
	uint8_t param_calib[22] = {0};

	BMP_SET_START_ADDR ( param_calib[0], BMP_CALIB_ADDR );
	//param_calib[0] = 0xAA
	//read calibration data
	HAL_I2C_Master_Transmit ( &hi2c1, BMP_WRITE_ADDR, param_calib, 1, 200 );
	//write
	HAL_I2C_Master_Receive ( &hi2c1, BMP_READ_ADDR, param_calib, 22, 200 );
	//read out EEPROM registers 16 bit MSB  First addres 0xAA

	calib_data->AC1 = param_calib[0] << 8 | param_calib[1];
	calib_data->AC2 = param_calib[2] << 8 | param_calib[3];
	calib_data->AC3 = param_calib[4] << 8 | param_calib[5];

	calib_data->AC4 = (uint16_t)(param_calib[6] << 8 | param_calib[7]);
	calib_data->AC5 = (uint16_t)(param_calib[8] << 8 | param_calib[9]);
	calib_data->AC6 = (uint16_t)(param_calib[10] << 8 | param_calib[11]);

	calib_data->B1 = param_calib[12] << 8 | param_calib[13];
	calib_data->B2 = param_calib[14] << 8 | param_calib[15];

	calib_data->MB = param_calib[16] << 8 | param_calib[17];
	calib_data->MC = param_calib[18] << 8 | param_calib[19];
	calib_data->MD = param_calib[20] << 8 | param_calib[21];
}

void set_oss(uint8_t val)
{
	if (val > 3)
	{
		val = 3;
	}
	dataBMP.oss = val;
}

void getParameterBMP(bmp_data_t	*dataBMP)
{
    ;
}


void bmp180_Init(bmp_data_t *dataBMP)
{
//	memset(dataBMP, 0, sizeof(dataBMP));
	set_oss(3);
	dataBMP->oss = read_oss();
	read_calib_data(&dataBMP->calib_data);

	//initializare nu trebuie de schimbat nimic

	getParameterBMP(dataBMP);
	dataBMP->flag_print = BMP180_PRINT_NOW;
}
                /*DO NOT USE*/
float get_temperature(bmp_data_t * bmp)
{   
    //NOTE: unsigned vars mess all up
    int32_t X1, X2, B5;
    float T;
    X1 = ( (read_UT() - _AC6) * _AC5)>>15; //X1=(UT-AC6)*AC5/2^15
    X2 = ((int16_t) _MC << 11)/(X1 + _MD); //X2=MC*2^11 / (X1+MD)
    B5 = X1+X2;
    T  = (B5+8)>>4;
    
    return T;
    
    /*DEBUG LINES FROM MAIN.C*/
//    int32_t X1, X2, B5;
//    float T;
//    X1 = ( (read_UT() - dataBMP.calib_data.AC6) * dataBMP.calib_data.AC5)>>15; //X1=(UT-AC6)*AC5/2^15
//    X2 = ((int16_t) dataBMP.calib_data.MC << 11)/(X1 + dataBMP.calib_data.MD); //X2=MC*2^11 / (X1+MD)
//    B5 = X1+X2;
//    T  = (B5+8)>>4;
//    
//    sprintf(uart_str_p, "X1 = (UT-AC6)*AC5/2^15 = (%u-%u) * %u/32768 = %u\n",read_UT(), dataBMP.calib_data.AC6, dataBMP.calib_data.AC5, X1);
//    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str_p), 100);
//    sprintf(uart_str_p, "X2 = MC*2^11 / (X1+MD) = %u*2048 / (%u+%u) = %u\n",(uint16_t) dataBMP.calib_data.MC, X1, dataBMP.calib_data.MD, X2);
//    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str_p), 100);
//    sprintf(uart_str_p, "B5 = X1+X2 = %u+%u = %u\n",X1, X2, B5);
//    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str_p), 100);
//    sprintf(uart_str_p, "T = (B5+8)/2^4 = (%u+8)>>4 = %3.1f\n",B5, T/10);
//    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str_p), 100);
}


bmp_data_t* calculate_TPA(bmp_data_t * bmp)
{
    //NOTE: unsigned vars mess all up
    int32_t X1, X2, B5;
    float T;
    X1 = ( (read_UT() - _AC6) * _AC5)>>15; //X1=(UT-AC6)*AC5/2^15
    X2 = ((int16_t) _MC << 11)/(X1 + _MD); //X2=MC*2^11 / (X1+MD)
    B5 = X1+X2;
    T  = (B5+8)>>4;    
bmp->temp = T;
    
    int32_t X3, B3, B6;
    uint32_t B4, B7; 
    float P;
    B6=B5-4000;
    X1 = ((_B2 * (B6 * (B6>>12))) >> 11);
    X2 = (_AC2*B6)>>11;
    X3 = X1+X2;
    B3 = (((_AC1 * 4 + X3) << bmp->oss) + 2) /4;
    X1 = ((_AC3 * B6) >> 11);
    X2 = ((_B1 * (B6 * (B6>>12))) >> 16);
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = _AC4 * (uint32_t)(X3 + 32768)>>15;
    B7 = ((uint32_t)read_UP()-B3)*(50000 >> bmp->oss);
    if (B7 < 0x80000000)
    {
        P = (B7*2)/B4;
    }
    else
    {
        P = (B7/B4)*2;
    }
    X1 = (P/256)*(P/256);
    X1 = (X1 * 3038)>>16;
    X2 = (-7357 * P)/65536;
    P = P + (X1 + X2 + 3791)/16;
bmp->pressure = P;
    
    float A;
    A = 44330 * (1.0 - pow(P / 101325, 0.1903));
bmp->altitude = A;
    return bmp;
}
