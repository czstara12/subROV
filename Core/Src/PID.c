/*
 * PIDCON.c
 *
 *  Created on: 2020年7月1日
 *      Author: starstory_星辰物语
 */
#include "PID.h"

#include "VRU.h"
//#include "remote.h"
#include "motor.h"
#include "deepSsensor.h"

//Roll Factor,Pitch Factor,Yaw Factor,Throttle Factor,Forward Factor,Lateral Factor

//kp ti dt 输出 前1次误差 前2次误差
float pid_ver[6][7] =
    {
        {0.03, 0.00004, 0.2},
        {-0.015, -0.00006, -0.3},
        {-0.003, 0, -0.1},
        {-0.1, -0.00001, 0},
        {1, 0, 0},
        {1, 0, 0},
};
//Roll,Pitch,Yaw,z x y
float target_ver[6];
int pidinit = 0;
void PID_init()
{
    target_ver[0] = 0;
    target_ver[1] = 0;
    target_ver[2] = yaw;
    target_ver[3] = 0.2;
    pidinit = 1;
}

void PID_CTRL() //综合控制 encodeL左侧编码器 encodeR右侧编码器
{
    //out += Kpr * (err - err1) + Tir * err+Tdr*(err-2*err1+err2); //
    //增量式PID方程 输出=输出+P*(本次误差-上次误差)+I*本次误差
    float err[6];
    float tmp_yaw;
    tmp_yaw = target_ver[2];
    if (tmp_yaw > 180)
    	tmp_yaw = target_ver[2] - 360;
    if (tmp_yaw < -180)
    	tmp_yaw = target_ver[2] + 360;

    err[0] = target_ver[0] - roll;  //求误差
    err[1] = target_ver[1] - pitch; //求误差
    err[2] = tmp_yaw - yaw;   //求误差

    if (err[2] > 180)
        err[2] -= 360;
    if (err[2] < -180)
        err[2] += 360;

    err[3] = target_ver[3] - deep; //求误差
    err[4] = target_ver[4]; //求误差
    err[5] = target_ver[5]; //求误差
    for (int i = 0; i < 6; i++)
    {
        pid_ver[i][3] += pid_ver[i][0] * (err[i] - pid_ver[i][4]) + pid_ver[i][1] * err[i] + pid_ver[i][2] * (err[i] - 2 * pid_ver[i][4] + pid_ver[i][5]);
        dof6[i] = pid_ver[i][3];       //输出到6自由度
        pid_ver[i][5] = pid_ver[i][4]; //更新上上次误差
        pid_ver[i][4] = err[i];        //更新上次误差
    }
}
