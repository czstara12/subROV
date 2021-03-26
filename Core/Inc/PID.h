/**
  ******************************************************************************
  * @file           : PIDCON.h
  * @brief          : PID控制的头文件
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
 * PIDCON.h
 *
 *  Created on: Jun 26, 2020
 *      Author: starstory_星辰物语
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

void PID_CTRL();
void PID_init();

//extern float Kpl, Til, Kpr, Tir;
extern int pidinit;
extern float target_ver[6];
float pid_ver[6][7];

#endif /* INC_PID_H_ */
