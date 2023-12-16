/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LSTD_TYPES.h"
#include "LBIT_MATH.h"
#include "ccd_interface.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BACK_MOTOR2_Pin GPIO_PIN_13
#define BACK_MOTOR2_GPIO_Port GPIOC
#define BACK_MOTOR_EN_Pin GPIO_PIN_0
#define BACK_MOTOR_EN_GPIO_Port GPIOA
#define ECCO4_Pin GPIO_PIN_1
#define ECCO4_GPIO_Port GPIOA
#define ECCO6_Pin GPIO_PIN_3
#define ECCO6_GPIO_Port GPIOA
#define BACK_MOTOR1_Pin GPIO_PIN_4
#define BACK_MOTOR1_GPIO_Port GPIOA
#define ECCO3_Pin GPIO_PIN_5
#define ECCO3_GPIO_Port GPIOA
#define TRIG1_Pin GPIO_PIN_6
#define TRIG1_GPIO_Port GPIOA
#define TRIG2_Pin GPIO_PIN_7
#define TRIG2_GPIO_Port GPIOA
#define TRIG3_Pin GPIO_PIN_0
#define TRIG3_GPIO_Port GPIOB
#define TRIG4_Pin GPIO_PIN_1
#define TRIG4_GPIO_Port GPIOB
#define ECCO5_Pin GPIO_PIN_10
#define ECCO5_GPIO_Port GPIOB
#define ECCO1_Pin GPIO_PIN_8
#define ECCO1_GPIO_Port GPIOA
#define ECCO2_Pin GPIO_PIN_9
#define ECCO2_GPIO_Port GPIOA
#define FRONT_MOTOR2_Pin GPIO_PIN_3
#define FRONT_MOTOR2_GPIO_Port GPIOB
#define TRIG5_Pin GPIO_PIN_7
#define TRIG5_GPIO_Port GPIOB
#define TRIG6_Pin GPIO_PIN_8
#define TRIG6_GPIO_Port GPIOB
#define FRONT_MOTOR1_Pin GPIO_PIN_9
#define FRONT_MOTOR1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
