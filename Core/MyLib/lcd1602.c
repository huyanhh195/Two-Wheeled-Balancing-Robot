/*
 * lcd1602.c
 *
 *  Created on: Dec 27, 2024
 *      Author: Huy
 */
#include "lcd1602.h"

// declare variable
static I2C_HandleTypeDef *hi2c_lcd;

// custom char
uint8_t bell[8]  = {0x4,0xe,0xe,0xe,0x1f,0x0,0x4};
uint8_t note[8]  = {0x2,0x3,0x2,0xe,0x1e,0xc,0x0};
uint8_t clock[8] = {0x0,0xe,0x15,0x17,0x11,0xe,0x0};
uint8_t heart[8] = {0x0,0xa,0x1f,0x1f,0xe,0x4,0x0};
uint8_t duck[8]  = {0x0,0xc,0x1d,0xf,0xf,0x6,0x0};
uint8_t check[8] = {0x0,0x1,0x3,0x16,0x1c,0x8,0x0};
uint8_t cross[8] = {0x0,0x1b,0xe,0x4,0xe,0x1b,0x0};
uint8_t retarrow[8] = {	0x1,0x1,0x5,0x9,0x1f,0x8,0x4};

// declare global variable
static uint8_t display_control = LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CUSOR_OFF | LCD_BLINK_OFF;
static uint8_t entry_mode = LCD_ENTRY_MODE | LCD_ENTRY_LEFT | LCD_ENTRYSHIFT_DECREMENT;

/**
 * @brief This function provides a time delay for LCD
 * @note This function use HAL_Delay for delay, can modify
 * @param delay is a delay duration in ms
 * @retval None
 */
__weak void lcd_delay(uint32_t delay)
{
  HAL_Delay(delay);
}

/**
 * @brief Sends a command or data to the LCD via I2C in 4-bit mode.
 * @param mode Specifies whether the input is a command (LCD_COMMAND) or data (LCD_DATA).
 * @param data The data to be sent 
 * @retval None
 */
static void lcd_write(uint8_t mode, char data)
{

  // initial buffer to send
  uint8_t buff[4] = {0};

  // Split the 8-bit data into high and low bit
  uint8_t data_h = 0, data_l = 0;
  data_h = data & 0XF0;
  data_l = (data << 4) & 0xF0;

  // select mode
  switch (mode)
  {
  case LCD_COMMAND:
  {
    // buffer for command
    buff[0] = data_h | LCD_COMMAND_ENABLE;  // en = 1, rs = 0 -> 0bxxxx1100
    buff[1] = data_h | LCD_COMMAND_DISABLE; // en = 0, rs = 0 -> 0bxxxx1000
    buff[2] = data_l | LCD_COMMAND_ENABLE;  // en = 1, rs = 0 -> 0bxxxx1100
    buff[3] = data_l | LCD_COMMAND_DISABLE; // en = 0, rs = 0 -> 0bxxxx1000
    HAL_I2C_Master_Transmit(hi2c_lcd, LCD_ADDR, buff, sizeof(buff), 100);
    break;
  }
  case LCD_DATA:
  {
    // buffer for command
    buff[0] = data_h | LCD_DATA_ENABLE;  // en = 1, rs = 1 -> 0bxxxx1101
    buff[1] = data_h | LCD_DATA_DISABLE; // en = 0, rs = 1 -> 0bxxxx1001
    buff[2] = data_l | LCD_DATA_ENABLE;  // en = 1, rs = 1 -> 0bxxxx1101
    buff[3] = data_l | LCD_DATA_DISABLE; // en = 0, rs = 1 -> 0bxxxx1001
    HAL_I2C_Master_Transmit(hi2c_lcd, LCD_ADDR, buff, sizeof(buff), 100);
    break;
  }
  }
}

/**
 * @brief Initializes the LCD in 4-bit mode via I2C communication.
 * @param hi2c Pointer to the I2C handle used for communication with the LCD.
 * @retval None
 */
void lcd_init(I2C_HandleTypeDef *hi2c)
{
  hi2c_lcd = hi2c;

  /* function set (initialization sequence for 4-bit mode) */
  lcd_delay(50);  // wait time > 15ms after VDD > 4.5V
  lcd_write(LCD_COMMAND, LCD_FUNCTION_SET | LCD_8BITMDOE);  //  function set(interface is 4 bits length)
  lcd_delay(1)  // wait time > 39us
  lcd_write(LCD_COMMAND, LCD_FUNCTION_SET | LCD_4BITMODE);  // function set(interface is 4 bits length)
  lcd_delay(1); // wait time > 39us
  lcd_write(LCD_COMMAND, LCD_FUNCTION_SET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS); // function set(interface is 4 bits length)
  lcd_delay(1); // wait time > 39us

  /* display config*/
  lcd_clear();  // dispay clear
  lcd_write(LCD_COMMAND, display_control);   // set display on/off control is 0x28
  lcd_delay(1); // wait time > 39us
  lcd_write(LCD_COMMAND, entry_mode);
  lcd_delay(1); // wait time
  //lcd_home();   // return cusor to home
	lcd_clear(); 
}

/**
 * @brief Sends a single character to the LCD for display.
 * @param data The character to be sent to the LCD.
 * @retval None
 */
void lcd_send_char(char data)
{
  lcd_write(LCD_DATA, data);
}

/**
 * @brief Sends a string to the LCD for display.
 * @param data The pointer points to string to be sent to the LCD.
 * @retval None
 */
void lcd_send_string(char *data)
{
  for (uint8_t i = 0; i < strlen(data); i++)
  {
    lcd_write(LCD_DATA, data[i]);
  }
}

/**
 * @brief Sends a int number to the LCD for display.
 * @param data The number to be sent to the LCD.
 * @retval None
 */
void lcd_send_int(uint8_t data)
{
  char buff[10] = {0};
  sprintf(buff, "%d", data);
  lcd_send_string(buff);
}

/**
 * @brief Sends a float number to the LCD for display.
 * @param data The number to be sent to the LCD.
 * @retval None
 */
void lcd_send_float(float data)
{
  char buff[10] = {0};
  sprintf(buff, "%.1f", data);
  lcd_send_string(buff);
}

/**
 * @brief Clears a specific position on the LCD by writing a blank space.
 * @param x The column position (0-based index).
 * @param y The row position (0-based index).
 * @retval None
 */
void lcd_clear_xy(uint8_t x, uint8_t y)
{
  lcd_goto_xy(x, y);
  lcd_write(LCD_DATA, BLANK_SPACE);
}

/**
 * @brief Displays a custom character at a specified position on the LCD.
 * @param x The column position (0-based index).
 * @param y The row position (0-based index).
 * @param location The CGRAM address (0–7) of the custom character.
 * @retval None
 */
void lcd_send_custom_char(int x, int y, int location){
  lcd_goto_xy(x,y);
  lcd_write(LCD_DATA, location);
	lcd_goto_xy(0,0);
}

/**
 * @brief Creates a custom character and stores it in the CGRAM of the LCD.
 * @param location The CGRAM address (0–7) to store the custom character.
 * @param char_map A pointer to an array of 8 bytes defining the custom character.
 *                 Each byte represents one row of the F5x8 mode.
 * @retval None
 */
void lcd_create_char(uint8_t location, uint8_t *char_map){
  // We only have 8 locations 0-7 for custom chars
  location &= 0x7; 
  // Set CRAM address
  lcd_write(LCD_COMMAND, LCD_SET_CRAM | (location << 3));
  lcd_delay(1);
  // write 8 byte custom char
  for(uint8_t i = 0; i < 8; i++){
    lcd_write(LCD_DATA, char_map[i]);
  }
}


/**
 * @brief Moves the LCD cursor to the specified (x, y) position.
 * @param x The column position (0-based index).
 * @param y The row position (0-based index).
 * @retval None
 */
void lcd_goto_xy(uint8_t x, uint8_t y)
{
  // display position DDRAM address
  uint8_t ddram_addr[] = {0x00, 0x40}; // row offset

  // position want to display
  uint8_t pos = y + ddram_addr[x];

  // send command
  lcd_write(LCD_COMMAND, LCD_SET_DDRAM | pos);
}

/**
 * @brief Clears the LCD display.
 * @param None
 * @retval None
 */
void lcd_clear()
{
  lcd_write(LCD_COMMAND, LCD_CLEAR_DISPLAY); // clear display
  lcd_delay(2);                              // wait time > 1.53ms
}

/**
 * @brief Moves the LCD cursor to the home position (top-left corner).
 *        This function returns the cursor to the original position if it has been shifted.
 * @param None
 * @retval None
 */
void lcd_home()
{
  lcd_write(LCD_COMMAND, LCD_RETURN_HOME); // return cursor to its original position if shifted. 
  lcd_delay(2); // wait time > 1.53ms
}

/**
 * @brief Controls the display on/off state of the LCD.
 *        This function enables or disables the display based on the input mode.
 * @param mode The mode to control the display:
 *             - DISPLAY_ON: Turns the display on.
 *             - DISPLAY_OFF: Turns the display off.
 * @retval None
 */
void lcd_display(uint8_t mode)
{
  switch (mode)
  {
  case DISPLAY_ON:
  {
    display_control = display_control | LCD_DISPLAY_ON;
    lcd_write(LCD_COMMAND, display_control);
    break;
  }
  case DISPLAY_OFF:
  {
    display_control = display_control & ~LCD_DISPLAY_ON;
    lcd_write(LCD_COMMAND, display_control);
    break;
  }
  }
  lcd_delay(1);
}

/**
 * @brief Controls the blinking of the LCD cursor.
 *        This function enables or disables the cursor blinking based on the input mode.
 * @param mode The mode to control the blinking:
 *             - BLINK_ON: Enables cursor blinking.
 *             - BLINK_OFF: Disables cursor blinking.
 * @retval None
 */
void lcd_blink(uint8_t mode)
{
  switch (mode)
  {
  case BLINK_ON:
  {
    display_control = display_control | LCD_BLINK_ON;
    lcd_write(LCD_COMMAND, display_control); // set display on/off control is 0x28                                                                             // wait time > 39us
    break;
  }
  case BLINK_OFF:
  {
    display_control = display_control & ~LCD_BLINK_ON;
    lcd_write(LCD_COMMAND, display_control); // set display on/off control is 0x28                                                                          // wait time > 39us
    break;
  }
  }
  lcd_delay(1);
}

/**
 * @brief Controls the visibility of the LCD cursor.
 *        This function enables or disables the cursor based on the input mode.
 * @param mode The mode to control the cursor visibility:
 *             - CUSOR_ON: Enables the cursor.
 *             - CUSOR_OFF: Disables the cursor.
 * @retval None
 */
void lcd_cusor(uint8_t mode)
{
  switch (mode)
  {
  case CUSOR_ON:
  {
    display_control |=  LCD_CUSOR_ON;
    lcd_write(LCD_COMMAND, display_control);
    break;
  }
  case CUSOR_OFF:
  {
    display_control &= ~LCD_CUSOR_ON;
    lcd_write(LCD_COMMAND, display_control);
    break;
  }
  }
  lcd_delay(1);
}
