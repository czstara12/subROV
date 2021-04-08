/*
 * raspi.h
 *
 *  Created on: Apr 3, 2021
 *      Author: czsta
 */

#ifndef INC_RASPI_H_
#define INC_RASPI_H_
#include "main.h"
void raspiInit(UART_HandleTypeDef *huart);
void raspi();
void raspierr(UART_HandleTypeDef *huart);

extern uint8_t raspiBuffer[32];
extern float raspich_float[4]; //树莓派通道
//0x5a 0xa5 Yaw Factor Throttle Factor Forward Factor Lateral Factor LED lock

#endif /* INC_RASPI_H_ */
