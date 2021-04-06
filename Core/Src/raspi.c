/*
 * raspi.c
 *
 *  Created on: Apr 3, 2021
 *      Author: czsta
 */


#include "raspi.h"
#include "motor.h"
#include "main.h"
uint8_t raspiBuffer[32] = { }; //树莓派包缓存
float raspich_float[4] = { }; //树莓派通道
//0x5a 0xa5 Yaw Factor Throttle Factor Forward Factor Lateral Factor LED lock
void raspiInit(UART_HandleTypeDef *huart)
{
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, raspiBuffer, 12);
}
//从中断触发 每次执行 将遥控器通道数据提取 归一到ch_float中
void raspi()
{
	short * date = (short *)(raspiBuffer + 2);
	for(int i=0;i<4;i++)
		raspich_float[i]=1.0*date[i]/0x7fff;

}
void raspierr()
{

}
