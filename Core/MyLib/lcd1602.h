#ifndef LCD_1602_H_
#define LCD_1602_H_

/* include lib */
#include <stdint.h>
#include <string.h>
#include "stm32f1xx_hal.h"

/*-------------------------MACRO-------------------------*/

// address of PCF8574 is 0 1 0 0 A2 A1 A0 R/W, default is 0 1 0 0 1 1 1 R/W
#define LCD_ADDR (0x4E) 

// mode of RS pin 
#define LCD_COMMAND 0
#define LCD_DATA 1

// strobe the EN pin of LCD
#define LCD_COMMAND_ENABLE 0x0C // 0bxxxx1100
#define LCD_COMMAND_DISABLE 0x08// 0bxxxx1000
#define LCD_DATA_ENABLE 0x0D    // 0bxxxx1101
#define LCD_DATA_DISABLE 0x09   // 0bxxxx1001

// command
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_CUSOR_SHIFT 0x10
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_CRAM 0x40
#define LCD_SET_DDRAM 0x80

// flag for entry mode command
#define LCD_ENTRY_RIGHT 0x00
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRYSHIFT_INCREMENT 0x01
#define LCD_ENTRYSHIFT_DECREMENT 0x00

// flag for display on/off control command
#define LCD_BLINK_OFF 0x00
#define LCD_BLINK_ON 0x01

#define LCD_CUSOR_OFF 0x00
#define LCD_CUSOR_ON 0x02

#define LCD_DISPLAY_OFF 0x00
#define LCD_DISPLAY_ON 0x04

// flags for display/cursor shift
#define LCD_DISPLAY_MOVE 0x08
#define LCD_CURSOR_MOVE 0x00
#define LCD_MOVE_RIGHT 0x04
#define LCD_MOVE_LEFT 0x00

// flag for function set command
#define LCD_4BITMODE 0x00
#define LCD_8BITMDOE 0x10

#define LCD_1LINE 0x00
#define LCD_2LINE 0x08

#define LCD_5x8DOTS 0x00
#define LCD_5x10DOTS 0x04

// status of blink
#define BLINK_OFF 0
#define BLINK_ON 1

/*-------------------------Prototype function-------------------------*/
void lcd_delay(uint32_t delay);
void lcd_write(uint8_t mode, uint8_t data);
void lcd_init(I2C_HandleTypeDef *hi2c);
void lcd_send_char(char data);
void lcd_send_string(char *data);
void lcd_clear(void);
void lcd_home(void);
void lcd_goto_xy(uint8_t x, uint8_t y);
void lcd_blink(uint8_t mode);
#endif

