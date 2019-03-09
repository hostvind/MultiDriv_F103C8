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
 * @file    LCD_I2C.c
 * @brief   LCD complex driver code.
 *
 * @addtogroup LCD
 * @{
 */

#include "lcd_i2c.h"



/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#define I2C_TIMEOUT                     100
#define I2C_INSTANCE                    lcdp->i2cp
#define I2C_ADDRESS                     lcdp->slaveaddress
#define LCD_BUSY_FLAG                   0X80

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   LCDD1 driver identifier.
 */
 //Shall NOT be declared here! Why the hell it was?!
//LCDDriver LCDD1;

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Write a data into a register for the lcd
 *
 * @param[in] lcdp          LCD driver
 * @param[in] reg           Register id
 * @param[in] value         Writing value
 *
 * @notapi
 */
static lcd_result lcdWriteRegister(LCDDriver *lcdp, uint8_t reg, uint8_t value) {
  uint8_t txbuf[4];
  HAL_Delay(2);

  txbuf[0] = reg | LCD_D_HIGHER(value) | LCD_E;
  if(lcdp->backlight)
    txbuf[0] |= LCD_K;
  txbuf[1] = reg | LCD_D_HIGHER(value);
  if(lcdp->backlight)
    txbuf[1] |= LCD_K;
  txbuf[2] = reg | LCD_D_LOWER(value) | LCD_E;
  if(lcdp->backlight)
    txbuf[2] |= LCD_K;
  txbuf[3] = reg | LCD_D_LOWER(value);
  if(lcdp->backlight)
    txbuf[3] |= LCD_K;

  if (HAL_I2C_Master_Transmit(I2C_INSTANCE, I2C_ADDRESS, txbuf, 4, I2C_TIMEOUT) == HAL_OK)
      return LCD_OK;
  else return LCD_NOK;
}

/**
 * @brief   Perform a initialization by instruction as explained in HD44780
 *          datasheet.
 * @note    This reset is required after a mis-configuration or if there aren't
 *          condition to enable internal reset circuit.
 *
 * @param[in] lcdp          LCD driver
 *
 * @notapi
 */
static lcd_result lcdInit(LCDDriver *lcdp) {
  uint8_t txbuf[2];
  HAL_Delay(50);
    //switch to 4Bit mode -- START --
  txbuf[0] = LCD_D(4) | LCD_D(5) | LCD_E;
  txbuf[1] = LCD_D(4) | LCD_D(5);
  if (HAL_I2C_Master_Transmit(I2C_INSTANCE, I2C_ADDRESS, txbuf, 2, I2C_TIMEOUT) != HAL_OK)
      return (LCD_NOK);
  HAL_Delay(5);

  txbuf[0] = LCD_D(4) | LCD_D(5) | LCD_E;
  txbuf[1] = LCD_D(4) | LCD_D(5);
  if (HAL_I2C_Master_Transmit(I2C_INSTANCE, I2C_ADDRESS, txbuf, 2, I2C_TIMEOUT) != HAL_OK)
      return (LCD_NOK);
  HAL_Delay(1);

  txbuf[0] = LCD_D(4) | LCD_D(5) | LCD_E;
  txbuf[1] = LCD_D(4) | LCD_D(5);
  if (HAL_I2C_Master_Transmit(I2C_INSTANCE, I2C_ADDRESS, txbuf, 2, I2C_TIMEOUT) != HAL_OK)
      return (LCD_NOK);
  HAL_Delay(1);

  txbuf[0] = LCD_D(5) | LCD_E;
  txbuf[1] = LCD_D(5);
  if (HAL_I2C_Master_Transmit(I2C_INSTANCE, I2C_ADDRESS, txbuf, 2, I2C_TIMEOUT) != HAL_OK)
      return (LCD_NOK);
  HAL_Delay(1);
    //switch to 4Bit mode -- END --
  /* Configuring data interface */
  if (lcdWriteRegister(lcdp, LCD_INSTRUCTION_R, LCD_FS | LCD_DATA_LENGTH |
                       lcdp->font | lcdp->lines) != LCD_OK)
      return (LCD_NOK);

  /* Turning off display and clearing */
  if (lcdWriteRegister(lcdp, LCD_INSTRUCTION_R, LCD_DC) != LCD_OK)
      return (LCD_NOK);
  if (lcdWriteRegister(lcdp, LCD_INSTRUCTION_R, LCD_CLEAR_DISPLAY) != LCD_OK)
      return (LCD_NOK);

  /* Setting display control turning on display */
  if (lcdWriteRegister(lcdp, LCD_INSTRUCTION_R, LCD_DC | LCD_DC_D |
                       lcdp->cursor | lcdp->blinking) != LCD_OK)
      return (LCD_NOK);

  /* Setting Entry Mode */
  if (lcdWriteRegister(lcdp, LCD_INSTRUCTION_R, LCD_EMS | LCD_EMS_ID) != LCD_OK)
      return (LCD_NOK);
  return LCD_OK;
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/


/**
 * @brief   Configures and activates the LCD Complex Driver  peripheral.
 * @note    You start with THIS ONE. It has some default values for you not to
            bother with looking for them and filling them in before init.
            If you further wish to change settings, simply change struct fields
            in driver and call lcdInit()
 *
 * @param[in] lcdp   pointer to the @p LCDDriver object
 * @param[in] i2cp   pointer to the @p I2C_HandleTypeDef object
 * @param[in] slaveaddress   is needed too. Trust me.
 *
 * @api
 */
lcd_result lcdStart(LCDDriver *lcdp, I2C_HandleTypeDef *i2cp, uint8_t slaveaddress)
    {

  if (lcdp == NULL)
      return LCD_NOK;

//  if ((lcdp->state == LCD_STOP) || (lcdp->state == LCD_ACTIVE))
//              return;// ("lcdStart(), invalid state");


  /* Initializing HD44780 by instructions. */
  lcdp->i2cp = i2cp;
  lcdp->slaveaddress = slaveaddress;
  lcdp->backlight = 1;
  lcdp->font = LCD_SET_FONT_5X8;
  lcdp->lines = LCD_SET_2LINES;
  lcdp->cursor = LCD_CURSOR_ON;
  lcdp->blinking = LCD_BLINKING_ON;
  if (lcdInit(lcdp) != LCD_OK)
      return (LCD_NOK);

  lcdp->state = LCD_ACTIVE;
  return LCD_OK;
}

/**
 * @brief   Deactivates the LCD Complex Driver  peripheral.
 *
 * @param[in] lcdp      pointer to the @p LCDDriver object
 *
 * @api
 */
lcd_result lcdStop(LCDDriver *lcdp) {

  if (lcdp == NULL)
      return LCD_NOK;

//  if ((lcdp->state == LCD_STOP) || (lcdp->state == LCD_ACTIVE))
//              return;// ("lcdStart(), invalid state");

  lcdp->backlight = 0;
  if (lcdWriteRegister(lcdp, LCD_INSTRUCTION_R, LCD_DC) != LCD_OK)
      return (LCD_NOK);
  if (lcdWriteRegister(lcdp, LCD_INSTRUCTION_R, LCD_CLEAR_DISPLAY) != LCD_OK)
      return (LCD_NOK);
  if (HAL_I2C_DeInit(I2C_INSTANCE) != HAL_OK)
      return (LCD_NOK); //was i2cStop
  lcdp->state = LCD_STOP;
  return LCD_OK;
}

/**
 * @brief   Turn on back-light.
 *
 * @param[in] lcdp      pointer to the @p LCDDriver object
 *
 * @api
 */
lcd_result lcdBacklightOn(LCDDriver *lcdp) {

  if (lcdp == NULL)
      return LCD_NOK;
//  if (lcdp->state == LCD_ACTIVE)
//        return;//      "lcdBacklightOn(), invalid state");

  lcdp->backlight = 1;
  return (lcdWriteRegister(lcdp, LCD_INSTRUCTION_R, 0));
}

/**
 * @brief   Turn off back-light.
 *
 * @param[in] lcdp      pointer to the @p LCDDriver object
 *
 * @api
 */
lcd_result lcdBacklightOff(LCDDriver *lcdp) {

  if (lcdp == NULL)
      return LCD_NOK;
  //osalDbgAssert((lcdp->state == LCD_ACTIVE), "lcdBacklightOff(), invalid state");

  lcdp->backlight = 0;
  return (lcdWriteRegister(lcdp, LCD_INSTRUCTION_R, 0));
}

/**
 * @brief   Clears display and return cursor in the first position.
 *
 * @param[in] lcdp      pointer to the @p LCDDriver object
 *
 * @api
 */
lcd_result lcdClearDisplay(LCDDriver *lcdp){

  if (lcdp == NULL)
      return LCD_NOK;
  //osalDbgAssert((lcdp->state == LCD_ACTIVE), "lcdClearDisplay(), invalid state");
  return (lcdWriteRegister(lcdp, LCD_INSTRUCTION_R, LCD_CLEAR_DISPLAY));
}

/**
 * @brief   Return cursor in the first position.
 *
 * @param[in] lcdp      pointer to the @p LCDDriver object
 *
 * @api
 */
lcd_result lcdReturnHome(LCDDriver *lcdp){

  if (lcdp == NULL)
      return LCD_NOK;
  //osalDbgAssert((lcdp->state == LCD_ACTIVE), "lcdReturnHome(), invalid state");
  return (lcdWriteRegister(lcdp, LCD_INSTRUCTION_R, LCD_RETURN_HOME));
}

/**
 * @brief   Set DDRAM address position leaving data unchanged.
 *
 * @param[in] lcdp      pointer to the @p LCDDriver object
 * @param[in] add       DDRAM address (from 0 to LCD_DDRAM_MAX_ADDRESS)
 *
 * @api
 */
lcd_result lcdSetAddress(LCDDriver *lcdp, uint8_t add){

  if (lcdp == NULL)
      return LCD_NOK;
  //osalDbgAssert((lcdp->state == LCD_ACTIVE),
                //"lcdSetAddress(), invalid state");
  if(add > LCD_SET_DDRAM_ADDRESS_MASK)
    return LCD_NOK;
  return (lcdWriteRegister(lcdp, LCD_INSTRUCTION_R, LCD_SET_DDRAM_ADDRESS | add));
}

/**
 * @brief   Writes string starting from a certain position.
 *
 * @detail  If string lenght exceeds, then is cutted
 *
 * @param[in] lcdp      pointer to the @p LCDDriver object
 * @param[in] string    string to write
 * @param[in] pos       position for cursor (from 0 to LCD_DDRAM_MAX_ADDRESS)
 *
 * @api
 */
lcd_result lcdCmd (LCDDriver *lcdp, uint8_t cmd)
{
      return (lcdWriteRegister(lcdp, LCD_INSTRUCTION_R, cmd));
}
lcd_result lcdPutChar (LCDDriver *lcdp, char ch)
{
      return (lcdWriteRegister(lcdp, LCD_DATA_R, ch));
}
lcd_result lcdPutString(LCDDriver *lcdp, char* string)
{
    if ((lcdp == NULL) || (string == NULL))
        return LCD_NOK;    
    if (lcdSetAddress(lcdp, 0) != LCD_OK) return (LCD_NOK);
    uint8_t i=0;
    while ((*string != '\0') && (i<0x68))
    {
        if (lcdWriteRegister(lcdp, LCD_DATA_R, *string) != LCD_OK)
            return (LCD_NOK);
        string++;
    }
    return LCD_OK;
}
lcd_result lcdWriteStringXY(LCDDriver *lcdp, char* string, uint8_t X, uint8_t Y)
    {
    if ((lcdp == NULL) || (string == NULL))
        return LCD_NOK;
    if (Y) Y = 1;
    uint8_t iteration;
    uint8_t pos = X+(0x40*Y);
  
  iteration = LCD_SET_DDRAM_ADDRESS_MASK - X + 1;
  if(iteration > 0)
    {
    if (lcdSetAddress(lcdp, pos) != LCD_OK)
        return (LCD_NOK);
    while((*string != '\0') && (iteration > 0))
        {
        if (lcdWriteRegister(lcdp, LCD_DATA_R, *string) != LCD_OK)
            return (LCD_NOK);
        string++;
        iteration--;
        }
    return LCD_OK;
    }
  else
    return LCD_NOK;
}

/**
 * @brief   Makes a shift according to an arbitrary direction
 *
 * @param[in] lcdp      pointer to the @p LCDDriver object
 * @param[in] dir       direction (LCD_RIGHT or LCD_LEFT)
 *
 * @api
 */
lcd_result lcdDoDisplayShift(LCDDriver *lcdp, uint8_t dir){

  if (lcdp == NULL)
      return LCD_NOK;
//  osalDbgAssert((lcdp->state == LCD_ACTIVE),
//                "lcdDoDisplayShift(), invalid state");
  return (lcdWriteRegister(lcdp, LCD_INSTRUCTION_R, LCD_CDS | LCD_CDS_SC | dir));
}

/** @} */

