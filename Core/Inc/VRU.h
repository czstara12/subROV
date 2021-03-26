/*
 * VRU.h
 *
 *  Created on: Jan 4, 2021
 *      Author: czsta
 */

#ifndef INC_VRU_H_
#define INC_VRU_H_

#include "main.h"

void VRUinit(UART_HandleTypeDef *huart);
void VRUupdate();
void VRUerror();
extern float pitch, roll, yaw;

#endif /* INC_VRU_H_ */
