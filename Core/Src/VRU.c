/*
 * VRU.c
 *
 *  Created on: Jan 4, 2021
 *      Author: czsta
 */

#include "VRU.h"
#include "PID.h"
#include "motor.h"

unsigned char ch110[82];
float pitch, roll, yaw,*dat;
int tim;
UART_HandleTypeDef *VRUhuart;

void VRUinit(UART_HandleTypeDef *huart)
{
	VRUhuart = huart;
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, ch110, 82);
}
//从串口中断触发 每次执行 提取姿态数据
void VRUupdate()
{
	float * datflo;
	datflo=(float *) (ch110 + 18);
	/*
	roll = *(float *) (ch110 + 54);
	pitch = *(float *) (ch110 + 58);
	yaw = *(float *) (ch110 + 62);
	*/
	roll = datflo[9];
	pitch = datflo[10];
	yaw = datflo[11];

	for(int i=0;i<16;i++)
	{
		frame.fdata[i] = datflo[i];
		//0加速度XYZ
		//3角速度XYZ
		//6磁强度XYZ
		//9欧拉角Roll Pitch Yaw
		//12四元数WXYZ
	}
    if(pidinit == 0)
        PID_init();
/*
    if(pidinit == 1)
    {
        PID_CTRL();
        setmotor();
    }*/
}
void VRUerror()
{
    HAL_UART_AbortReceive(VRUhuart);
    HAL_UART_Receive_DMA(VRUhuart, ch110, 82);
}
