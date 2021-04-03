/**
 ******************************************************************************
 * @file           : remote.c
 * @brief          : S.Bus遥控器驱动
 ******************************************************************************
 * @attention
 *
 * &copy; Copyright (c) 2020 智茂科技.
 * All rights reserved.
 *
 * This software component is licensed by 智茂科技 under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/*
 * remote.c
 *
 *  Created on: Aug 24, 2020
 *      Author: starStory_星辰物语
 */
#include "remote.h"
#include "motor.h"
#include "PID.h"
#include "main.h"
uint8_t remoteBuffer[25] = { }; //遥控器包缓存
float ch_float[6] = { }; //遥控器通道
//Roll Factor,Pitch Factor,Yaw Factor,Throttle Factor,Forward Factor,Lateral Factor
float conf=0;
float val=0;
void remoteInit(UART_HandleTypeDef *huart)
{
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, remoteBuffer, 25);
}
//从中断触发 每次执行 将遥控器通道数据提取 归一到ch_float中
void remote(UART_HandleTypeDef *huart)
{
	float *dat = (float*)(remoteBuffer+4);
	if(remoteBuffer[3]==0x00)
	{
		ch_float[4]=dat[1];
		ch_float[5]=dat[0];
	}else if(remoteBuffer[3]==0x01)
	{
		ch_float[0]=dat[0];
		ch_float[1]=dat[1];
	}else if(remoteBuffer[3]==0x02)
	{
		lock = remoteBuffer[7];
	}else if(remoteBuffer[3]==0x03)//通道选择
	{
		conf=dat[0];
		int i_conf=(int)conf;
		val=pid_ver[i_conf/3][i_conf%3];
		frame.fdata[24]=conf;
		frame.fdata[25]=val;
	}else if(remoteBuffer[3]==0x04)//通道值
	{
		val=dat[0];
		int i_conf=(int)conf;
		pid_ver[i_conf/3][i_conf%3]=val;
		frame.fdata[25]=val;
	}else if(remoteBuffer[3]==0x05)
	{
		ch_float[2]=dat[0];
		ch_float[3]=dat[1];
	}
}
