/*
 * lib_lcd.h
 *
 *  Created on: Feb 18, 2021
 *      Author: utilisateur
 */

#ifndef INC_LIB_LCD_H_
#define INC_LIB_LCD_H_

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"

// Device I2C Adress
#define LCD_ADDRESS     0x7c
#define RGB_ADDRESS     0xc4

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// color define
#define WHITE           0
#define RED             1
#define GREEN           2
#define BLUE            3

#define REG_RED         0x04        // pwm2
#define REG_GREEN       0x03        // pwm1
#define REG_BLUE        0x02        // pwm0

#define REG_MODE1       0x00
#define REG_MODE2       0x01
#define REG_OUTPUT      0x08


// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

typedef struct
{
  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;
} rgb_lcd;

void lcd_init(I2C_HandleTypeDef* I2Cx, rgb_lcd* DataStruct);
void lcd_write(I2C_HandleTypeDef* I2Cx, uint8_t value);
void lcd_print(I2C_HandleTypeDef* I2Cx, char *str);
void lcd_position(I2C_HandleTypeDef* I2Cx,char col, char row);
void clearlcd(void);
void reglagecouleur(uint8_t R,uint8_t G,uint8_t B);
void variable_Transmit(uint16_t address, uint8_t *Data, uint16_t len);




#endif /* INC_LIB_LCD_H_ */
