# LCD_16_02_I2C
## Table of Contents:
* [General info](#general-info)
* [Stogage LCD](#function-description)
## General info
This project was developed to implement LCD 1602 with 4 bit via I2C protocol

## Function Description:
The LCD display Module is built in a LSI controller, the controller has two 8-bit registers, an 
instruction register (IR) and a data register (DR)
The IR stores instruction codes, such as display clear and cursor shift, and address information for 
display data RAM (DDRAM) and character generator (CGRAM). The IR can only be written from the 
MPU. The DR temporarily stores data to be written or read from DDRAM or CGRAM. When 
address information is written into the IR, then data is stored into the DR from DDRAM or CGRAM. 
By the register selector (RS) signal, these two registers can be selected.
### Busy Flag (BF)
When the busy flag is 1, the controller LSI is in the internal operation mode, and the next instruction 
will not be accepted. When RS=0 and R/W=1, the busy flag is output to DB7. The next instruction 
must be written after ensuring that the busy flag is 0

### Address Counter (AC)
The address counter (AC) assigns addresses to both DDRAM and CGRAM

### Display Data RAM (DDRAM)
This DDRAM is used to store the display data represented in 8-bit character codes. Its extended 
capacity is 80ǘ8 bits or 80 characters. Below figure is the relationship between DDRAM addresses 
and positions on the liquid crystal display

### Character Generator ROM (CGROM)
The CGROM generate 5ǘ8 dot or 5ǘ10 dot character patterns from 8-bit character codes.

### Character Generator RAM (CGRAM)
In CGRAM, the user can rewrite character by program. For 5ǘ8 dots, eight character patterns can be 
written, and for 5ǘ10 dots, four character patterns can be written.
Write into DDRAM the character code at the addresses shown as the left column of table 1. To show 
the character patterns stored in CGRAM.

## Pinout of the PCF8574
* P0 is connected to the pin RS of the LCD. This RS pin is defines whether the transmitted byte is a command (0) or Data (1).
* P1 is connected to the R/W pin of the LCD. This pin should be LOW when writing the data to the LCD and HIGH when reading the data from the LCD.
* P2 is connected to the Enable pin of the LCD. This pin is used for the strobe (E=1 & E=0).
* P3 is connected to the Backlight of the Display. Setting this pin to 1 will turn the backlight ON.
* P4 – P7 are connected to the data pins D4 – D7. Since only 4 data pins are available in the PCF8574, we need to configure the LCD in the 4bit Mode.

## When the display powers up, it is configured as follows:
'''
1. Display clear
2. Function set: 
   DL = 1; 8-bit interface data 
   N = 0; 1-line display 
   F = 0; 5x8 dot character font 
3. Display on/off control: 
   D = 0; Display off 
   C = 0; Cursor off 
   B = 0; Blinking off 
4. Entry mode set: 
   I/D = 1; Increment by 1
   S = 0; No shift 
'''
