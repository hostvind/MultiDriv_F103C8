/*
 * bmp180.h
 *
 *  Created on: 11 feb. 2019
 *      Author: gheorghe
 */

#ifndef BMP180_H_
#define BMP180_H_

#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "math.h"


#define CHIP_ID			                    (0xD0)
#define BMP_CTRL_MEAS_REG	                (0xF4)
#define BMP_READ_ADDR	                    (0xEF)
#define BMP_WRITE_ADDR		                (0xEE)
#define BMP_CALIB_ADDR  	                (0xAA)
#define BMP_PRESS_OSS_0 	                (0x34)
#define BMP_PRESS_OSS_1 	                (0x74)
#define BMP_PRESS_OSS_2 	                (0xB4)
#define BMP_PRESS_OSS_3 	                (0xF4)
#define BMP_START_TEMP		                (0x2E)
#define BMP_START_PRESS		                (0x34)
#define BMP_DATA_MSB_ADDR	                (0xF6)
#define BMP_DATA_LSB_ADDR	                (0xF7)
#define BMP_DATA_XLSB_ADDR	                (0xF8)
#define BMP_SOFT_RESET  	                (0xB6)
#define BMP_SOFT_RESET_REG 	                (0xE0)
#define BMP_PRESS_CONST_SEA_LEVEL 	    	(1013.25F)
#define BMP_PRESS_CONST_COEFICIENT 	    	(44330)
#define BMP_PRESS_CONST_EXPONENT 	    	(0.1902949F)

#define BMP_COMPUTE_DATA(bmp_data, unit)    (bmp_data / unit)
#define BMP_COMPUTE_PRESS				    (100.0f)
#define BMP_COMPUTE_TEMP				    (10.0f)

#define BMP_SET_START_ADDR(var, addr)       (var = addr)
#define FUNC_A_PROD_B_CIT_C(A, B, C)		((A) * (B) / (C))

#define READ_OSS(val)						((val & 0xC0) >> 6)
#define SEA_LVL								(1013.25f)

#define GET_CELSIUS(var)					(var / 10)
#define GET_KELVIN(var)						((var / 10) + 273)
#define GET_FAHRENHEIT(var)					((var * 0.18) + 32)

#define GET_BAR_PRESURE(var)				(var / 1000)
#define GET_MM_HG_PRESURE(var)				(var * 0.750061)
#define GET_H_PASCAL_PRESURE(var)			(var / 100)

#define GET_METER(var)						(var)
#define GET_FEET(var)						(var / 0.3048)
#define GET_CENTIMETER(var)					(var * 100)

#define UM_CELSIUS							0
#define UM_KELVIN							1
#define UM_FAHRENHEIT						2

#define UM_BAR_PRESURE						0
#define UM_MM_HG_PRESURE					1
#define UM_H_PASCAL_PRESURE					2

#define UM_METER							0
#define UM_FEET								1
#define UM_CENTIMETER						2

#define BMP_2_POW_11						(1<<11)

#define BMP180_GET_TEMPERATURE				0
#define BMP180_GET_PRESSURE					1
#define BMP180_GET_ALTITUDE					2

#define BMP180_OPTION_TEMPERATURE				0
#define BMP180_OPTION_PRESSURE					1
#define BMP180_OPTION_ALTITUDE					2


#define BMP180_PRINT_NOW					1
#define BMP180_DO_NOT_PRINT					0


// #define BMP180_UNITATE_MASURA_TEMP			bmp_data_t->				


typedef struct
{
	int16_t	 AC1;
	int16_t  AC2;
	int16_t  AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t  B1;
	int16_t  B2;
	int16_t  MB;
	int16_t  MC;
	int16_t  MD;
} bmp_calib_param_t;

typedef struct
{
	bmp_calib_param_t	calib_data;
	float temp;
	float altitude;
	float pressure;
	int32_t UP;
	int32_t UT;
	int32_t B5;
	int16_t oss;

	uint8_t menu;
	uint8_t unitateMasuraTemp;
	uint8_t unitateMasuraPresure;
	uint8_t unitateMasuraAltitude;
	uint8_t unitateMasuraOption;

	uint8_t type_BMP;
	uint8_t flag_regim;//pentru selectarea optiunilor mod normal functionare, mod alegere unitate de masura

	uint8_t flag_print;


} bmp_data_t;

enum overampling_ratio
{
	OSS0 = 1,
	OSS1,
	OSS2,
	OSS3
};

#endif /* BMP180_H_ */
//read function-------------------
void read_calib_data(bmp_calib_param_t *calib_data);
int32_t read_UT(void);
int32_t read_UP(void);
//read function++++++++++++++++++++++++

//calc function-------------------
int32_t read_oss(void);

//calc function++++++++++++++++++++++++

//init function-------------------
void bmp180_Init(bmp_data_t *dataBMP);

//init function++++++++++++++++++++++++

//get function-------------------
void getParameterBMP(bmp_data_t	*dataBMP);
float get_temperature(bmp_data_t * bmp);            //DO NOT USE
float get_pressure(bmp_calib_param_t * calib_data); //DO NOT USE
bmp_data_t* calculate_TPA(bmp_data_t * bmp);
float get_altitude(void);
float get_ValueBMP180(uint8_t option);
int32_t get_B5(void);
int32_t get_UT(void);
int32_t get_UP(void);
//get functionn++++++++++++++++++++++++

//set function-------------------
void set_oss(uint8_t val);
//set function++++++++++++++++++++++++



