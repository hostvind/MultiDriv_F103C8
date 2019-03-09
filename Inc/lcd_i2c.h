/*
    PLAY Embedded - Copyright (C) 2006..2015 Rocco Marco Guglielmi

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
    Special thanks to Giovanni Di Sirio for teachings, his moral support and
    friendship. Note that some or every piece of this file could be part of
    the ChibiOS project that is intellectual property of Giovanni Di Sirio.
    Please refer to ChibiOS/RT license before use this file.
	
	For suggestion or Bug report - guglielmir@playembedded.org
 */

/**
 * @file    LCD_I2C.h
 * @brief   LCD Complex Driver header.
 *
 * @addtogroup LCD
 * @{
 */

#ifndef LCD_I2C_H_
#define LCD_I2C_H_

#include "stm32f1xx_hal.h"
//#include "userconf.h"



/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/


/* LCD BITS */
#define LCD_RS                          1 << 0
#define LCD_RW                          1 << 1
#define LCD_E                           1 << 2
#define LCD_K                           1 << 3

#define LCD_D(n)                        1 << n
#define LCD_D_HIGHER(n)                 (n & 0xF0)
#define LCD_D_LOWER(n)                  ((n & 0x0F) << 4)

/* LCD REGISTERS */
#define LCD_INSTRUCTION_R               0
#define LCD_DATA_R                      LCD_RS


/* LCD_INSTRUCTIONS */
#define LCD_CLEAR_DISPLAY               0x01    

#define LCD_RETURN_HOME                 0x02    

#define LCD_EMS                         0x04    //screen and cursor settings
#define LCD_EMS_S                       0x01    //screen shift on input
#define LCD_EMS_ID                      0x02    //1 - increment. 0 - decrement

#define LCD_DC                          0x08    //display control
#define LCD_DC_B                        0x01    //cursor blink
#define LCD_DC_C                        0x02    //cursor underscore
#define LCD_DC_D                        0x04    //display ON

#define LCD_CDS                         0x10    //cursor-display shift
#define LCD_CDS_RL                      0x04    //1 - right. 0 - left
#define LCD_CDS_SC                      0x08    //0 - shift cursor. 1 - screen

#define LCD_FS                          0x20    //font select
#define LCD_FS_F                        0x04    //0 - 5x8 dots. 1 - 5x10 (rare)
#define LCD_FS_N                        0x08    //0 - 1 row, 1 - 2 rows used
#define LCD_FS_DL                       0x10    //1 - 8 bit data bus. 0 - 4 bits

#define LCD_SET_CGRAM_ADDRESS           0x40
#define LCD_SET_CGRAM_ADDRESS_MASK      0X3F

#define LCD_SET_DDRAM_ADDRESS           0x80
#define LCD_SET_DDRAM_ADDRESS_MASK      0X7F

/**
 * @brief   Generic definition of right direction
 */
#define   LCD_RIGHT                     0x04

/**
 * @brief   Generic definition of left direction
 */
#define   LCD_LEFT                      0x00

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#define LINE_DATA_LEN                   4
#define LCD_DATA_LENGTH                 0x00
#define LCD_BL_ON                       TRUE
#define LCD_BL_OFF                      FALSE



/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @name    lcd data structures and types
 * @{ */

/**
 * @brief  LCD response
 */
typedef enum {
  LCD_OK = 0x00,
  LCD_NOK = 0x01,
} lcd_result;

/**
 * @brief  LCD cursor control
 */
typedef enum {
  LCD_CURSOR_OFF = 0x00,
  LCD_CURSOR_ON = 0x02,
} lcd_cursor_t;

/**
 * @brief  LCD blinking control
 */
typedef enum {
  LCD_BLINKING_OFF = 0x00,
  LCD_BLINKING_ON = 0x01,
} lcd_blinking_t;

/**
 * @brief  LCD display settings
 */
typedef enum {
  LCD_SET_FONT_5X8 = 0x00,
  LCD_SET_FONT_5X10 = 0x04
} lcd_set_font_t;

/**
 * @brief  LCD display settings
 */
typedef enum {
  LCD_SET_1LINE = 0x00,
  LCD_SET_2LINES = 0x08
} lcd_set_lines_t;
/** @}  */

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  LCD_UNINIT = 0,
  LCD_STOP = 1,
  LCD_ACTIVE = 2,
} lcd_state_t;

/**
 * @brief   Structure representing an LCD driver.
 */
typedef struct {
  /**
   * @brief Driver state.
   */
  lcd_state_t        state;
  /**
   * @brief  Current Back-light status.
   *
   */  
  uint8_t backlight;
  /**
   * @brief  Pointer to the I2C driver used by this driver
   */
  I2C_HandleTypeDef *i2cp;
  /**
   * @brief  I2C slave address
   */
  uint8_t slaveaddress;
  /**
   * @brief  LCD cursor control
   */
  lcd_cursor_t cursor;
  /**
   * @brief  LCD blinking control
   */
  lcd_blinking_t blinking;
  /**
   * @brief  LCD font settings
   */
  lcd_set_font_t font;
  /**
   * @brief  LCD lines settings
   */
  lcd_set_lines_t lines;
} LCDDriver;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern LCDDriver LCDD1;

#ifdef __cplusplus
extern "C" {
#endif
  lcd_result lcdInit(LCDDriver *lcdp);
  lcd_result lcdStart(LCDDriver *lcdp, I2C_HandleTypeDef *i2cp, uint8_t slaveaddress);
  lcd_result lcdStop(LCDDriver *lcdp);
  lcd_result lcdBacklightOn(LCDDriver *lcdp);
  lcd_result lcdBacklightOff(LCDDriver *lcdp);
  lcd_result lcdClearDisplay(LCDDriver *lcdp);
  lcd_result lcdReturnHome(LCDDriver *lcdp);
  lcd_result lcdSetAddress(LCDDriver *lcdp, uint8_t add);
  lcd_result lcdCmd (LCDDriver *lcdp, uint8_t cmd);
  lcd_result lcdPutChar (LCDDriver *lcdp, char ch);
  lcd_result lcdPutString(LCDDriver *lcdp, char* string);
  lcd_result lcdWriteStringXY(LCDDriver *lcdp, char* string, uint8_t X, uint8_t Y);
  lcd_result lcdDoDisplayShift(LCDDriver *lcdp, uint8_t dir);
#ifdef __cplusplus
}
#endif

#endif /* _LCD_I2C_H_ */

/** @} */
