

#include "LCD_Interface.h"





u8 PositionCursor_x=0,PositionCursor_y=0;

#if LCD_MODE==_8_BIT

static void WriteIns(u8 ins)
{
	HAL_GPIO_WritePin(RS,GPIO_PIN_RESET);
	DIO_WritePort(LCD_PORT,ins);
	HAL_GPIO_WritePin(EN,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(EN,GPIO_PIN_RESET);
	HAL_Delay(1);
	
}

static void WriteData(u8 data)
{
	HAL_GPIO_WritePin(RS,GPIO_PIN_SET);
	DIO_WritePort(LCD_PORT,data);
	HAL_GPIO_WritePin(EN,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(EN,GPIO_PIN_RESET);
	HAL_Delay(1);
	
}


void LCD_Init(void)
{
	HAL_Delay(50);
	WriteIns(0x38);
	WriteIns(CURSOR_MODE);
	WriteIns(0x01);//clear screen
	HAL_Delay(1);
	WriteIns(0x06);

}
#else

static void WriteIns(u8 ins)
{
	HAL_GPIO_WritePin(RS,GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(D7,GET_BIT(ins,7));
	HAL_GPIO_WritePin(D6,GET_BIT(ins,6));
	HAL_GPIO_WritePin(D5,GET_BIT(ins,5));
	HAL_GPIO_WritePin(D4,GET_BIT(ins,4));
	
	
	HAL_GPIO_WritePin(EN,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(EN,GPIO_PIN_RESET);
	HAL_Delay(1);
	
	HAL_GPIO_WritePin(D7,GET_BIT(ins,3));
	HAL_GPIO_WritePin(D6,GET_BIT(ins,2));
	HAL_GPIO_WritePin(D5,GET_BIT(ins,1));
	HAL_GPIO_WritePin(D4,GET_BIT(ins,0));
	
	
	HAL_GPIO_WritePin(EN,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(EN,GPIO_PIN_RESET);
	HAL_Delay(1);
	
}

static void WriteData(u8 data)
{
	HAL_GPIO_WritePin(RS,GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(D7,GET_BIT(data,7));
	HAL_GPIO_WritePin(D6,GET_BIT(data,6));
	HAL_GPIO_WritePin(D5,GET_BIT(data,5));
	HAL_GPIO_WritePin(D4,GET_BIT(data,4));
	
	HAL_GPIO_WritePin(EN,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(EN,GPIO_PIN_RESET);
	HAL_Delay(1);
	
	HAL_GPIO_WritePin(D7,GET_BIT(data,3));
	HAL_GPIO_WritePin(D6,GET_BIT(data,2));
	HAL_GPIO_WritePin(D5,GET_BIT(data,1));
	HAL_GPIO_WritePin(D4,GET_BIT(data,0));
	
	
	HAL_GPIO_WritePin(EN,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(EN,GPIO_PIN_RESET);
	HAL_Delay(1);
	
}


void LCD_Init(void)
{
	//for lcd 16*2

	HAL_Delay(50);
	WriteIns(0x02);
	WriteIns(0x28);
	WriteIns(CURSOR_MODE);
	WriteIns(0x01);//clear screen
	HAL_Delay(1);
	WriteIns(0x06);


	///for lcd 20*4
	/*
	HAL_Delay(15);
	WriteIns(0x02);
	HAL_Delay(2);
	WriteIns(_LCD_4BIT_MODE);
	HAL_Delay(2);
	WriteIns(0x0c);
	WriteIns(0x06);
	WriteIns(_LCD_CLEAR);
	HAL_Delay(2);
	*/
}

#endif

void LCD_GoTo(u8 row,u8 col)
{

	///   for lcd 16*2


	PositionCursor_x=row;
	PositionCursor_y=col;
	if (row==0)
	{
		WriteIns(0x80+col);
	}
	else if (row==1)
	{
		WriteIns(0x80+0x40+col);
	}



	/// for lcd 20*4
	/*
	col-=1;
	if (row==1)
	{
		WriteIns(0x80+col);
	}
	else if (row==2)
	{
		WriteIns(0x80+0x40+col);
	}
	else if (row==3)
	{
		WriteIns(0x80+20+col);
	}
	else
	{
		WriteIns(0x80+0x40+20+col);
	}
	
	PositionCursor_y = col+1;
	PositionCursor_x = row;
	*/
}

void LCD_GoBackCells(u8 NoOfCells)
{
	PositionCursor_y=PositionCursor_y-NoOfCells;
	LCD_GoTo(PositionCursor_x,PositionCursor_y);
}


void LCD_Clear(void)
{
// 	WriteIns(0x01);//clear screen
// 	HAL_Delay(1);
	LCD_GoTo(0,0);
	LCD_WriteString("                ");
	LCD_GoTo(1,0);
	LCD_WriteString("                ");
	LCD_GoTo(0,0);
}
void DisplayShift_Right(void)
{

	WriteIns(0x1c);
}



void LCD_WriteChar(u8 ch)
{
// 	if(y>15&&x==0)
// 	{
// 		LCD_GoTo(1,0);
// 		y=0;
// 		x=1;
// 	}
// 	else if (y>15&&x==1)
// 	{
// 		LCD_GoTo(0,0);
// 		y=0;
// 		x=0;
// 	}
// 	else
// 	{
// 		//do nothing
// 	}
	
	WriteData(ch);
	PositionCursor_y++;
}


void LCD_WriteString(c8*str)
{
	
	u8 i;
	for (i=0;str[i];i++)
	{
		LCD_WriteChar(str[i]);
	}
	
}


void LCD_WriteStringGoTo(u8 line,u8 cell,c8*str)
{
	LCD_GoTo(line,cell);
	
	LCD_WriteString(str);
	
}


void LCD_ClearPosition(u8 line,u8 cell,u8 NoCells)
{
	LCD_GoTo(line,cell);
	while(NoCells)
	{
		LCD_WriteChar(' ');
		NoCells--;
	}
}

void LCD_WriteNumber (s32 num)
{
	u8 str[20];
	s8 i=0;
	if (num==0)
	{
		LCD_WriteChar('0');
	}
	if (num<0)
	{
		LCD_WriteChar('-');
		num*=-1;
	}
	while(num)
	{
		str[i]=num%10  + '0';
		i++;
		num=num/10;
	}
	for(i=i-1;i>=0;i--)
	{
		LCD_WriteChar(str[i]);
	}
	
	
}


void LCD_WriteBinary(u8 num)
{
	s8 i=7;
	for (;i>=0;i--)
	{
		if ((num>>i)&1)
		{
			LCD_WriteChar('1');
		}
		else
		{
			LCD_WriteChar('0');
		}
				
	}
	
	
}



void LCD_WriteHex(u16 num)
{
	u8 str[20];
	s8 i=0;
	if (num==0)
	{
		LCD_WriteString("00");
	}
	while(num)
	{
		u8 temp=num%16;
		if (temp<=9)
		{
			str[i]=temp+ '0' ;
		}
		else
		{
			str[i]=(temp-10)+'A';
		}
		num/=16;
		i++;
	}
	for(i=i-1;i>=0;i--)
	{
		LCD_WriteChar(str[i]);
	}
	
	
	
}





void LCD_Write_4D(u16 num)
{
	LCD_WriteChar(((num%10000)/1000)+'0');
	LCD_WriteChar(((num%1000)/100)+'0');
	LCD_WriteChar(((num%100)/10)+'0');
	LCD_WriteChar(((num%10)/1)+'0');
	
}

void LCD_CustomChar(u8 adress, u8* pattern)
{
	
	WriteIns(0x40+adress*8);
	
	for (int i=0; i<8; i++)
	{
		WriteData(pattern[i]);
	}
	
	
	WriteIns(0x80);

	
}
