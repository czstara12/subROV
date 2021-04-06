/**
  ******************************************************************************
  * @file           : remote.h
  * @brief          : S.Bus遥控器驱动头文件
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
 * remote.h
 *
 *  Created on: Aug 24, 2020
 *      Author: starStory_星辰物语
 */

#ifndef INC_REMOTE_H_
#define INC_REMOTE_H_

#include "main.h"

void remoteInit(UART_HandleTypeDef *huart);
void remote(uint8_t * remoteBuffer);
void remoteerr();

extern float ch_float[6];//遥控器通道
extern float conf;
extern float val;

#endif /* INC_REMOTE_H_ */
