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
void remoteerr(UART_HandleTypeDef *huart);

extern float ch_float[6];//遥控器通道
extern float conf;
extern float val;

#endif /* INC_REMOTE_H_ */
