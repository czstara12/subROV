/*
 * raspi.c
 *
 *  Created on: Apr 3, 2021
 *      Author: czsta
 */


#include "raspi.h"
#include "motor.h"
#include "PID.h"
#include "VRU.h"
uint8_t raspiBuffer[32] = { }; //树莓派包缓存
float raspich_float[4] = { }; //树莓派通道
//0x5a 0xa5 Yaw Factor Throttle Factor Forward Factor Lateral Factor LED lock



void raspiInit(UART_HandleTypeDef *huart)
{
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, raspiBuffer, 12);
}
//从中断触发 每次执行 将遥控器通道数据提取 归一到ch_float中
void raspiUpdate()
{
	short * date = (short *)(raspiBuffer + 2);
	for(int i=0;i<4;i++)
		raspich_float[i]=1.0*date[i]/0x7fff;

    target_ver[2] = yaw + raspich_float[0]*180;
    target_ver[3] = raspich_float[1];
    target_ver[4] = raspich_float[2];
    target_ver[5] = raspich_float[3];
    if(raspiBuffer[10]==0)
    	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    else
    	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    lock=raspiBuffer[11];

    frame.fdata[28]=target_ver[2];
    frame.fdata[29]=target_ver[3];
    frame.fdata[30]=target_ver[4];
    frame.fdata[31]=target_ver[5];//回传三轴数据目标
}
void raspierr(UART_HandleTypeDef *huart)
{
    HAL_UART_AbortReceive(huart);
    HAL_UART_Receive_DMA(huart, raspiBuffer, 12);
}
