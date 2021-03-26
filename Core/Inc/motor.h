/*
 * motor.h
 *
 *  Created on: Aug 24, 2020
 *      Author: starStory_星辰物语
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define MOTOR_1 TIM4->CCR1
#define MOTOR_2 TIM4->CCR2
#define MOTOR_3 TIM4->CCR3
#define MOTOR_4 TIM4->CCR4
#define MOTOR_5 TIM8->CCR1
#define MOTOR_6 TIM8->CCR2
#define MOTOR_7 TIM8->CCR3
#define MOTOR_8 TIM8->CCR4

#define MOTOR_CENTER 1500
#define MOTOR_WAY 800

//void motorInit(TIM_HandleTypeDef *htim);
void setmotor();
void motorinit();
extern float dof6[6];
extern int lock;

#endif /* INC_MOTOR_H_ */
