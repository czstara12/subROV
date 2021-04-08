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
