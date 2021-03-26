/*
 * motor.c
 *
 *  Created on: Aug 24, 2020
 *      Author: starStory_星辰物语
 */
#include "main.h"
//#include "remote.h"
#include "motor.h"
//Roll Factor,Pitch Factor,Yaw Factor,Throttle Factor,Forward Factor,Lateral Factor
float hcc[8][6] =
{
    {0,              0,              1.0f,           0,                  -1.0f,              1.0f, },
    {0,              0,              -1.0f,          0,                  -1.0f,              -1.0f,},
    {0,              0,              -1.0f,          0,                  1.0f,               1.0f, },
    {0,              0,              1.0f,            0,                  1.0f,              -1.0f,},
    {1.0f,           -1.0f,          0,              -1.0f,              0,                  0,    },
    {-1.0f,          -1.0f,          0,              -1.0f,              0,                  0,    },
    {1.0f,           1.0f,           0,              -1.0f,              0,                  0,    },
    {-1.0f,          1.0f,           0,              -1.0f,              0,                  0,    }
};
float dof6[6] = {};
int lock = 0;
/*
//roll pitch yawA yawB Throt ForA ForB LatA LatB
float hcc[8][9] = {
		{ 0, 0, 1.0f, 0, 0, -1.0f, 0, 0, 1.0f, },
		{ 0, 0, 0, -1.0f,0, -1.0f, 0, -1.0f, 0, },
		{ 0, 0, 0, -1.0f, 0, 0, 1.0f, 0, 1.0f, },
		{ 0,0, 1.0f, 0, 0, 0, 1.0f, -1.0f, 0, },
		{ 1.0f, -1.0f, 0, 0, 1.0f, 0, 0,0, 0, },
		{ -1.0f, -1.0f, 0, 0, 1.0f, 0, 0, 0, 0, },
		{ 1.0f, 1.0f, 0, 0,1.0f, 0, 0, 0, 0, },
		{ -1.0f, 1.0f, 0, 0, 1.0f, 0, 0, 0, 0, }, };*/
float motor[8];
//每次执行会将六自由度dof6转换为8个电机通道 并赋值
void motorinit()
{
	extern TIM_HandleTypeDef htim4;
	extern TIM_HandleTypeDef htim8;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}
void setmotor()
{
    int time = HAL_GetTick();
    for (int i = 0; i < 8; i++)
    {
        float tmp = 0;
        for (int j = 0; j < 6; j++)
            tmp += hcc[i][j] * dof6[j];
        if(tmp > 1)
            tmp = 1;
        if(tmp < -1)
            tmp = -1;
        if(time<10000||lock==0)//初始化||没有解开锁定
			motor[i] = 0;
        else
        	motor[i] = tmp;
        frame.fdata[i + 16] = tmp;//回传数据
    }

    MOTOR_1 = motor[0] * MOTOR_WAY + MOTOR_CENTER;
    MOTOR_2 = motor[1] * MOTOR_WAY + MOTOR_CENTER;
    MOTOR_3 = motor[2] * MOTOR_WAY + MOTOR_CENTER;
    MOTOR_4 = motor[3] * MOTOR_WAY + MOTOR_CENTER;
    MOTOR_5 = motor[4] * MOTOR_WAY + MOTOR_CENTER;
    MOTOR_6 = motor[5] * MOTOR_WAY + MOTOR_CENTER;
    MOTOR_7 = motor[6] * MOTOR_WAY + MOTOR_CENTER;
    MOTOR_8 = motor[7] * MOTOR_WAY + MOTOR_CENTER;
}
