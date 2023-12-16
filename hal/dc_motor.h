/*
 * dc_motor.h
 *
 *  Created on: Nov 21, 2023
 *      Author: msame
 */

#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

#include "main.h"

#include "dc_motor_cfg.h"


typedef enum
{
	STOP_SPEED	 = 4096,
	LOW_SPEED  	 = 1650,
	MEDIUM_SPEED = 1000,
	HIGH_SPEED 	 = 500
//	LOW_SPEED  	 = 2500,
//	MEDIUM_SPEED = 1000,
//	HIGH_SPEED 	 = 0

}MotorSpeed_t;






void motor_vidInit(void);
void motor_vidMoveForward(const MotorSpeed_t speed);
void motor_vidMoveBackward(const MotorSpeed_t speed);
void motor_vidStop(void);
void motor_vidMoveForwardRight(const MotorSpeed_t speed);
void motor_vidMoveForwardLeft(const MotorSpeed_t speed);
void motor_vidMoveBackwardRight(const MotorSpeed_t speed);
void motor_vidMoveBackwardLeft(const MotorSpeed_t speed);
void motor_vidTurnWheelsLeft(void);
void motor_vidTurnWheelsRight(void);



#endif /* DC_MOTOR_H_ */
