/*
 * LCD_Interface.h
 *
 * Created: 6/12/2023 8:50:59 PM
 *  Author: msame
 */ 

#include "main.h"
#include "LCD_Cfg.h"


#ifndef LCD_INTERFACE_H_
#define LCD_INTERFACE_H_



typedef enum{
	_LCD_CLEAR=0x01,
	_LCD_CURSOR_OFF=0x0C,
	_LCD_CURSOR_ON=0x0F,
	_LCD_4BIT_MODE=0x28,
	_LCD_8BIT_MODE=0x38,
	_LCD_ON=0x0F,
	_LCD_CURSOR_UNDERLINE=0x0E,
	_LCD_CURSOR_SHIFT_LEFT=0x10,
	_LCD_CURSOR_SHIFT_RIGHT=0x14,
	_LCD_CURSOR_INCREMENT=0x06,
	_LCD_CGRAM_START_ADDRESS=0x40
}lcd_cmd_t;




void LCD_Init();

	
void LCD_Clear();

/*move in LCD*/
void LCD_GoTo(u8 line,u8 cell);
void LCD_GoBackCells(u8 NoOfCells);

void DisplayShift_Right(void);

void LCD_WriteString(c8*str);
void LCD_WriteStringGoTo(u8 line,u8 cell,c8*str);
void LCD_ClearPosition(u8 line,u8 cell,u8 NoCells);



void LCD_WriteNumber(s32 num);
void LCD_WriteBinary(u8 num);
void LCD_WriteHex(u16 num);


void LCD_WriteChar(u8 ch);


void LCD_Write_4D(u16 num);


void LCD_CustomChar(u8 adress, u8* pattern);


#endif /* LCD_INTERFACE_H_ */
