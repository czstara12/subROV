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
#include "VRU.h"
uint8_t remoteBuffer[32] = { }; //遥控器包缓存
float ch_float[6] = { }; //遥控器通道
//Roll Factor,Pitch Factor,Yaw Factor,Throttle Factor,Forward Factor,Lateral Factor
int i_conf;
float val=0;
float yawa = 0;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
void remoteInit(UART_HandleTypeDef *huart)
{
    if(&huart1==huart)
	{
		HAL_UART_Receive_DMA(huart, remoteBuffer+16, 25);
	}else if(&huart5==huart)
	{
		HAL_UART_Receive_DMA(huart, remoteBuffer, 25);
	}
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}
//从中断触发 每次执行 将遥控器通道数据提取 归一到ch_float中
void remoteUpdate(uint8_t * remoteBuffer)
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
		i_conf=(int)dat[0];
		val=pid_ver[i_conf/3][i_conf%3];
		frame.fdata[24]=dat[0];
		frame.fdata[25]=val;
	}else if(remoteBuffer[3]==0x04)//通道值
	{
		val=dat[0];
		pid_ver[i_conf/3][i_conf%3]=val;
		frame.fdata[25]=val;
	}else if(remoteBuffer[3]==0x05)
	{
		ch_float[2]=dat[0];
		//ch_float[3]=dat[1];
	}else if(remoteBuffer[3]==0x06)
	{
		ch_float[3]=dat[0];
	}
    target_ver[0] = ch_float[0]*30;
    target_ver[1] = ch_float[1]*30;
    yawa = 0.02*ch_float[2]*30;
    target_ver[3] = ch_float[3];
    target_ver[4] = ch_float[4];
    target_ver[5] = ch_float[5];
    frame.fdata[26]=target_ver[i_conf/3];
}
void remoteerr(UART_HandleTypeDef *huart)
{
    //HAL_UART_AbortReceive(huart);
    if(&huart1==huart)
	{
		HAL_UART_Receive_DMA(huart, remoteBuffer+16, 25);
	}else if(&huart5==huart)
	{
		HAL_UART_Receive_DMA(huart, remoteBuffer, 25);
	}
    //HAL_UART_Receive_DMA(huart, remoteBuffer, 16);
}
