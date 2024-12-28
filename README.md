# LCD_16_02_I2C
## Table of Contents:
* [General info](#general-info)
* [Stogage LCD](#stogage-lcd)
## General info
This project was developed to implement LCD 1602 via I2C protocol

## Stogage LCD:
 - DISPLAY DATA RAM (DDRAM)
 - Character Generator ROM (CG ROM)
 - Character Generator RAM (CG RAM)

## Process write data to LCD1602:
```
Step 1: Write RS pin to 0 if send command, 1 if send data
Step 2: Write 0 to R/W pin to write data
Step 3: Write data
Step 4: Write 1 to EN pin to latch data, then write 0  
```
