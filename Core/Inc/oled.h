#ifndef __OLED_H
#define __OLED_H			  	 

#include "main.h"
//-----------------OLED端口定义---------------- 
#define OLED_RST_Clr() HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_RESET)//PA15 RESET
#define OLED_RST_Set() HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET)

#define OLED_RS_Clr() HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)//PB3 DC
#define OLED_RS_Set() HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)

#define OLED_SCLK_Clr() HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET) //PB5 CLK
#define OLED_SCLK_Set() HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET)

#define OLED_SDIN_Clr() HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_RESET) //PB4 MOSI
#define OLED_SDIN_Set() HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_SET)

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
//OLED控制用函数
typedef unsigned char u8;
typedef unsigned long u32;

void OLED_WR_Byte(u8 dat,u8 cmd);	         //向OLED写入一个字节
void OLED_Display_On(void);                //开启OLED显示
void OLED_Display_Off(void);               //关闭OLED显示
void OLED_Refresh_Gram(void);		   				 //刷新显示
void OLED_Init(void);                      //初始化OLED
void OLED_Clear(void);                     //OLED清屏，清空后整个屏幕都是黑色
void OLED_DrawPoint(u8 x,u8 y,u8 t);       //OLED画点
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode); //在指定的位置显示一个字符
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size); //在指定的位置显示数字
void OLED_ShowString(u8 x,u8 y,const char *p);	          //在指定的位置显示字符串
void Oled_Show(void);
#endif     
	 
