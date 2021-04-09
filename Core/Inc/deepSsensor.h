/*
 * deepSsensor.h
 *
 *  Created on: Apr 9, 2021
 *      Author: czsta
 */

#ifndef INC_DEEPSSENSOR_H_
#define INC_DEEPSSENSOR_H_


extern unsigned char deepSsensorBuff[20];
extern float deep,temperature;

void deepSensorInit(UART_HandleTypeDef *huart);
void deepSensorUpdate(unsigned char *deepSsensorBuff);
void deepSensorerr(UART_HandleTypeDef *huart);

#endif /* INC_DEEPSSENSOR_H_ */
