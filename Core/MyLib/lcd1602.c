#include "lcd1602.h"

// declare variable
static I2C_HandleTypeDef *hi2c_lcd;

void lcd_delay(uint32_t delay){
  HAL_Delay(delay);
}

static void lcd_write(uint8_t mode, uint8_t data)
{

  // initial buff to send
  uint8_t buff[4] = {0};

  // using the LCD in 4 bit mode, so send the same data twice
  uint8_t data_h = 0;
  uint8_t data_l = 0;

  // split 4 bit 
  data_h = data & 0XF0;
  data_l = (data << 4) & 0xF0;

  // select mode
  switch (mode)
  {
  case LCD_COMMAND:{
    buff[0] = data_h | LCD_COMMAND_ENABLE; // en = 1, rs = 0 -> 0bxxxx1100
    buff[1] = data_h | LCD_COMMAND_DISABLE;// en = 0, rs = 0 -> 0bxxxx1000
    buff[2] = data_l | LCD_COMMAND_ENABLE; // en = 1, rs = 0 -> 0bxxxx1100
    buff[3] = data_l | LCD_COMMAND_DISABLE;// en = 0, rs = 0 -> 0bxxxx1000
    HAL_I2C_Master_Transmit(hi2c_lcd, LCD_ADDR, buff, sizeof(buff), 100);
    break;
  }
  case LCD_DATA:{
    buff[0] = data_h | LCD_DATA_ENABLE;    // en = 1, rs = 1 -> 0bxxxx1101
    buff[1] = data_h | LCD_DATA_DISABLE;   // en = 0, rs = 1 -> 0bxxxx1001
    buff[2] = data_l | LCD_DATA_ENABLE; 	 // en = 1, rs = 1 -> 0bxxxx1101
    buff[3] = data_l | LCD_DATA_DISABLE;   // en = 0, rs = 1 -> 0bxxxx1001
    HAL_I2C_Master_Transmit(hi2c_lcd, LCD_ADDR, buff, sizeof(buff), 100);
    break;}
  }
}

// lcd 4 bit interface
void lcd_init(I2C_HandleTypeDef *hi2c){
	hi2c_lcd = hi2c;
	
	/* function set */
  lcd_delay(50);	// wait time > 15ms after VDD > 4.5V
  lcd_write(LCD_COMMAND, LCD_FUNCTION_SET | LCD_8BITMDOE); //  function set(interface is 4 bits length)
  lcd_delay(1);		// wait time > 39us
  lcd_write(LCD_COMMAND, LCD_FUNCTION_SET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS); // function set(interface is 4 bits length)
  lcd_delay(1);		// wait time > 39us
	lcd_write(LCD_COMMAND, LCD_FUNCTION_SET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS); // function set(interface is 4 bits length)
  lcd_delay(1);		// wait time > 39us

  /* display */
  lcd_write(LCD_COMMAND, LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CUSOR_OFF | LCD_BLINK_OFF); // set display on/off control is 0x28 
	lcd_delay(1);		// wait time > 39us
  lcd_clear();// dispay clear
	lcd_delay(2);		// wait time > 1.53ms
  lcd_write(LCD_COMMAND, LCD_ENTRY_MODE | LCD_ENTRY_LEFT | LCD_ENTRYSHIFT_DECREMENT); 
	lcd_delay(1);		// wait time 
	lcd_home(); // return cusor to home
	lcd_delay(2);
	 lcd_clear();// dispay clear
	lcd_delay(2);		// wait time > 1.53ms
}

void lcd_send_char(char data){
  lcd_write(LCD_DATA, data);
}

void lcd_send_string(char *data){
	for(uint8_t i = 0; i < strlen(data); i++){
		lcd_write(LCD_DATA, data[i]);
	}
}

//Write “00H” to DDRAM and set DDRAM address to “00H” from AC
void lcd_clear(){
	lcd_write(LCD_COMMAND, LCD_CLEAR_DISPLAY); // clear display
	lcd_delay(2);		// wait time > 1.53ms
}

void lcd_goto_xy(uint8_t x, uint8_t y){
	// display position DDRAM address
	uint8_t y_offset[] = {0x00, 0x40}; // row offset 
	
	// position want to display
	uint8_t pos = y | y_offset[--x];
	
	// send command
	lcd_write(LCD_COMMAND, LCD_SET_DDRAM | pos);
}

// Set DDRAM address to “00H” from AC and return cursor to its original position if shifted. The contents of DDRAM are not changed
void lcd_home(){
	lcd_write(LCD_COMMAND, LCD_RETURN_HOME);
	lcd_delay(2);		// wait time > 1.53ms
}

void lcd_blink(uint8_t mode){
	switch(mode){
		case BLINK_ON:{
		  lcd_write(LCD_COMMAND, LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CUSOR_ON | LCD_BLINK_ON); // set display on/off control is 0x28 
	lcd_delay(1);		// wait time > 39us
			break;
		}
		case BLINK_OFF:
					  lcd_write(LCD_COMMAND, LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CUSOR_OFF | LCD_BLINK_OFF); // set display on/off control is 0x28 
	lcd_delay(1);		// wait time > 39us
			break;
	}
}
