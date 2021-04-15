/*
 * deepSsensor.h
 *
 *  Created on: Apr 9, 2021
 *      Author: czsta
 */

#ifndef INC_DEEPSENSOR_H_
#define INC_DEEPSENSOR_H_

#include "main.h"

extern unsigned char deepSsensorBuff[20];
extern float deep,temperature;

void deepSensorInit(UART_HandleTypeDef *huart);
void deepSensorUpdate(unsigned char *deepSsensorBuff);
void deepSensorerr(UART_HandleTypeDef *huart);

#endif /* INC_DEEPSENSOR_H_ */
