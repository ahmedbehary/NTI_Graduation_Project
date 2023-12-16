/*
 * dc_motor.c
 *
 *  Created on: Nov 21, 2023
 *      Author: msame
 */


#include "dc_motor.h"



extern TIM_HandleTypeDef htim5;



void motor_vidInit(void){

	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);

	/* move motor enable cfg */
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,STOP_SPEED);

	/* turn motor enable cfg */
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,STOP_SPEED);

	/* move motor cfg */
	HAL_GPIO_WritePin(BACK_MOTOR_S1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BACK_MOTOR_S2,GPIO_PIN_SET);

	/* turn motor cfg */
	HAL_GPIO_WritePin(FRONT_MOTOR_S1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FRONT_MOTOR_S2,GPIO_PIN_RESET);


}

void motor_vidMoveForward(const MotorSpeed_t speed){


	/* move motor enable cfg */
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,speed);

	/* turn motor enable cfg */
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,STOP_SPEED);


	/* move motor cfg */
	HAL_GPIO_WritePin(BACK_MOTOR_S1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BACK_MOTOR_S2,GPIO_PIN_RESET);

	/* turn motor cfg */
	HAL_GPIO_WritePin(FRONT_MOTOR_S1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FRONT_MOTOR_S2,GPIO_PIN_RESET);


}

void motor_vidMoveForwardRight(const MotorSpeed_t speed){


	/* move motor enable cfg */
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,speed);

	/* turn motor enable cfg */
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,HIGH_SPEED);

	/* move motor cfg */
	HAL_GPIO_WritePin(BACK_MOTOR_S1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BACK_MOTOR_S2,GPIO_PIN_RESET);

	/* turn motor cfg */
	HAL_GPIO_WritePin(FRONT_MOTOR_S1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FRONT_MOTOR_S2,GPIO_PIN_SET);


}

void motor_vidMoveForwardLeft(const MotorSpeed_t speed){

	/* move motor enable cfg */
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,speed);

	/* turn motor enable cfg */
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,HIGH_SPEED);

	/* move motor cfg */
	HAL_GPIO_WritePin(BACK_MOTOR_S1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BACK_MOTOR_S2,GPIO_PIN_RESET);

	/* turn motor cfg */
	HAL_GPIO_WritePin(FRONT_MOTOR_S1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(FRONT_MOTOR_S2,GPIO_PIN_RESET);


}

void motor_vidMoveBackward(const MotorSpeed_t speed){

	/* move motor enable cfg */
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,speed);

	/* turn motor enable cfg */
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,STOP_SPEED);

	/* move motor cfg */
		HAL_GPIO_WritePin(BACK_MOTOR_S1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BACK_MOTOR_S2,GPIO_PIN_SET);

		/* turn motor cfg */
		HAL_GPIO_WritePin(FRONT_MOTOR_S1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(FRONT_MOTOR_S2,GPIO_PIN_RESET);


}


void motor_vidMoveBackwardRight(const MotorSpeed_t speed){

	/* move motor enable cfg */
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,speed);

	/* turn motor enable cfg */
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,HIGH_SPEED);

	/* move motor cfg */
		HAL_GPIO_WritePin(BACK_MOTOR_S1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BACK_MOTOR_S2,GPIO_PIN_SET);

		/* turn motor cfg */
		HAL_GPIO_WritePin(FRONT_MOTOR_S1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(FRONT_MOTOR_S2,GPIO_PIN_SET);


}



void motor_vidMoveBackwardLeft(const MotorSpeed_t speed){

	/* move motor enable cfg */
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,speed);

	/* turn motor enable cfg */
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,HIGH_SPEED);

	/* move motor cfg */
		HAL_GPIO_WritePin(BACK_MOTOR_S1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BACK_MOTOR_S2,GPIO_PIN_SET);

		/* turn motor cfg */
		HAL_GPIO_WritePin(FRONT_MOTOR_S1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(FRONT_MOTOR_S2,GPIO_PIN_RESET);


}




void motor_vidTurnWheelsRight(void)
{


	/* move motor enable cfg */
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,STOP_SPEED);

	/* turn motor enable cfg */
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,HIGH_SPEED);

	/* move motor cfg */
	HAL_GPIO_WritePin(BACK_MOTOR_S1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BACK_MOTOR_S2,GPIO_PIN_SET);

	/* turn motor cfg */
	HAL_GPIO_WritePin(FRONT_MOTOR_S1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FRONT_MOTOR_S2,GPIO_PIN_SET);


}




void motor_vidTurnWheelsLeft(void)
{


	/* move motor enable cfg */
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,STOP_SPEED);

	/* turn motor enable cfg */
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,HIGH_SPEED);

	/* move motor cfg */
	HAL_GPIO_WritePin(BACK_MOTOR_S1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BACK_MOTOR_S2,GPIO_PIN_SET);

	/* turn motor cfg */
	HAL_GPIO_WritePin(FRONT_MOTOR_S1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(FRONT_MOTOR_S2,GPIO_PIN_RESET);


}




void motor_vidStop(void){

	/* move motor enable cfg */
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,STOP_SPEED);
	/* turn motor enable cfg */
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,STOP_SPEED);

	/* move motor cfg */
		HAL_GPIO_WritePin(BACK_MOTOR_S1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(BACK_MOTOR_S2,GPIO_PIN_SET);
		/* turn motor cfg */
		HAL_GPIO_WritePin(FRONT_MOTOR_S1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(FRONT_MOTOR_S2,GPIO_PIN_RESET);

}































