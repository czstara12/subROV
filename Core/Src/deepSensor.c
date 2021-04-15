/*
 * deepSsensor.c
 *
 *  Created on: Apr 9, 2021
 *      Author: czsta
 */
#include <stdio.h>
#include "oled.h"

unsigned char deepSensorBuff[20];
float deep,temperature;

void deepSensorInit(UART_HandleTypeDef *huart)
{
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, deepSensorBuff, 20);
}

void deepSensorUpdate(unsigned char *deepSsensorBuff)
{
	sscanf((char *)deepSensorBuff,"T=%fD=%f",&temperature,&deep);
	if(deep<0)deep=0;
	frame.fdata[27]=deep;
	OLED_ShowNumber(64, 36, (int)(temperature*100), 3, 12);
	//frame.fdata[28]=temperature;
}

void deepSensorerr(UART_HandleTypeDef *huart)
{
    HAL_UART_AbortReceive(huart);
    HAL_UART_Receive_DMA(huart, deepSensorBuff, 20);
}
