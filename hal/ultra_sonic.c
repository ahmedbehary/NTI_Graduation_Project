/*
 * ultra_sonic.c
 *
 *  Created on: Nov 30, 2023
 *      Author: msame
 */


#include "ultra_sonic_interface.h"




extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;






void Ultrasonic_vidInit(void)
{

	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
//	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
//
//	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
//	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
//	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
//	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);




//	__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
//	__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC2);
//	__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);

//	HAL_TIM_IC_Stop_IT(htim, Channel)



	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
//	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,4);

//	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,4);
//	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,4);


}





void Ultrasonic_vidTrigger (Ultra_sonic_t Sensor_type)
{
	switch(Sensor_type)
	{
	case FORWARD_SENSOR:

		HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		delay(10);  // wait for 10 us
		HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

		__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);

		break;

	case FORWARD_LEFT_SENSOR:

		HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		delay(10);  // wait for 10 us
		HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

		__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);

		break;

	case FORWARD_RIGHT_SENSOR:

		HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		delay(10);  // wait for 10 us
		HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

		__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);

		break;

	case BACKWARD_SENSOR:

		HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		delay(10);  // wait for 10 us
		HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

		__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);

		break;

	case BACKWARD_LEFT_SENSOR:

		HAL_GPIO_WritePin(TRIG5_GPIO_Port, TRIG5_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		delay(10);  // wait for 10 us
		HAL_GPIO_WritePin(TRIG5_GPIO_Port, TRIG5_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

		__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC3);

		break;

	case BACKWARD_RIGHT_SENSOR:

		HAL_GPIO_WritePin(TRIG6_GPIO_Port, TRIG6_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		delay(10);  // wait for 10 us
		HAL_GPIO_WritePin(TRIG6_GPIO_Port, TRIG6_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

		__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC4);

		break;




	}


}








