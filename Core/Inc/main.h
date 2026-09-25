/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC_FR_A_Pin GPIO_PIN_2
#define ENC_FR_A_GPIO_Port GPIOE
#define ENC_FR_B_Pin GPIO_PIN_3
#define ENC_FR_B_GPIO_Port GPIOE
#define MOTOR_FL_PWM_Pin GPIO_PIN_0
#define MOTOR_FL_PWM_GPIO_Port GPIOC
#define MOTOR_FR_PWM_Pin GPIO_PIN_1
#define MOTOR_FR_PWM_GPIO_Port GPIOC
#define MOTOR_RL_PWM_Pin GPIO_PIN_2
#define MOTOR_RL_PWM_GPIO_Port GPIOC
#define MOTOR_RR_PWM_Pin GPIO_PIN_3
#define MOTOR_RR_PWM_GPIO_Port GPIOC
#define ENC_FL_A_Pin GPIO_PIN_0
#define ENC_FL_A_GPIO_Port GPIOA
#define ENC_FL_B_Pin GPIO_PIN_1
#define ENC_FL_B_GPIO_Port GPIOA
#define ENC_RR_A_Pin GPIO_PIN_2
#define ENC_RR_A_GPIO_Port GPIOB
#define ENC_RL_A_Pin GPIO_PIN_12
#define ENC_RL_A_GPIO_Port GPIOD
#define ENC_RL_B_Pin GPIO_PIN_13
#define ENC_RL_B_GPIO_Port GPIOD
#define MOTOR_FL_DIR_Pin GPIO_PIN_6
#define MOTOR_FL_DIR_GPIO_Port GPIOC
#define MOTOR_FR_DIR_Pin GPIO_PIN_7
#define MOTOR_FR_DIR_GPIO_Port GPIOC
#define MOTOR_RL_DIR_Pin GPIO_PIN_8
#define MOTOR_RL_DIR_GPIO_Port GPIOC
#define MOTOR_RR_DIR_Pin GPIO_PIN_9
#define MOTOR_RR_DIR_GPIO_Port GPIOC
#define ENC_RR_B_Pin GPIO_PIN_12
#define ENC_RR_B_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
