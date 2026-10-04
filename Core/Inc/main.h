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
#define MOTOR_FL_DIR_Pin GPIO_PIN_2
#define MOTOR_FL_DIR_GPIO_Port GPIOE
#define MOTOR_FR_DIR_Pin GPIO_PIN_3
#define MOTOR_FR_DIR_GPIO_Port GPIOE
#define MOTOR_RL_DIR_Pin GPIO_PIN_4
#define MOTOR_RL_DIR_GPIO_Port GPIOE
#define MOTOR_RR_DIR_Pin GPIO_PIN_5
#define MOTOR_RR_DIR_GPIO_Port GPIOE
#define MOTOR_FL_PWM_Pin GPIO_PIN_0
#define MOTOR_FL_PWM_GPIO_Port GPIOC
#define MOTOR_FR_PWM_Pin GPIO_PIN_1
#define MOTOR_FR_PWM_GPIO_Port GPIOC
#define MOTOR_RL_PWM_Pin GPIO_PIN_2
#define MOTOR_RL_PWM_GPIO_Port GPIOC
#define MOTOR_RR_PWM_Pin GPIO_PIN_3
#define MOTOR_RR_PWM_GPIO_Port GPIOC
#define ENC_RR_A_Pin GPIO_PIN_0
#define ENC_RR_A_GPIO_Port GPIOA
#define ENC_RR_B_Pin GPIO_PIN_1
#define ENC_RR_B_GPIO_Port GPIOA
#define ENC_FR_B_Pin GPIO_PIN_4
#define ENC_FR_B_GPIO_Port GPIOA
#define ENC_FR_A_Pin GPIO_PIN_6
#define ENC_FR_A_GPIO_Port GPIOA
#define LED_RUN_Pin GPIO_PIN_7
#define LED_RUN_GPIO_Port GPIOE
#define LED_LINK_Pin GPIO_PIN_8
#define LED_LINK_GPIO_Port GPIOE
#define LED_FAULT_Pin GPIO_PIN_9
#define LED_FAULT_GPIO_Port GPIOE
#define ESTOPN_Pin GPIO_PIN_10
#define ESTOPN_GPIO_Port GPIOE
#define FL_FAULT_Pin GPIO_PIN_12
#define FL_FAULT_GPIO_Port GPIOE
#define FR_FAULT_Pin GPIO_PIN_13
#define FR_FAULT_GPIO_Port GPIOE
#define RL_FAULT_Pin GPIO_PIN_14
#define RL_FAULT_GPIO_Port GPIOE
#define RR_FAULT_Pin GPIO_PIN_15
#define RR_FAULT_GPIO_Port GPIOE
#define Servo_PWM_5_Pin GPIO_PIN_14
#define Servo_PWM_5_GPIO_Port GPIOB
#define Servo_PWM_6_Pin GPIO_PIN_15
#define Servo_PWM_6_GPIO_Port GPIOB
#define ENC_FL_A_Pin GPIO_PIN_12
#define ENC_FL_A_GPIO_Port GPIOD
#define ENC_FL_B_Pin GPIO_PIN_13
#define ENC_FL_B_GPIO_Port GPIOD
#define Servo_PWM_1_Pin GPIO_PIN_6
#define Servo_PWM_1_GPIO_Port GPIOC
#define Servo_PWM_2_Pin GPIO_PIN_7
#define Servo_PWM_2_GPIO_Port GPIOC
#define Servo_PWM_3_Pin GPIO_PIN_8
#define Servo_PWM_3_GPIO_Port GPIOC
#define Servo_PWM_4_Pin GPIO_PIN_9
#define Servo_PWM_4_GPIO_Port GPIOC
#define OLED_INA226_SDA_Pin GPIO_PIN_8
#define OLED_INA226_SDA_GPIO_Port GPIOA
#define OLED_INA226_SCL_Pin GPIO_PIN_9
#define OLED_INA226_SCL_GPIO_Port GPIOA
#define NRF_CSN_Pin GPIO_PIN_10
#define NRF_CSN_GPIO_Port GPIOA
#define NRF_CE_Pin GPIO_PIN_11
#define NRF_CE_GPIO_Port GPIOA
#define BMI_ACC_CS_Pin GPIO_PIN_10
#define BMI_ACC_CS_GPIO_Port GPIOC
#define BMI_GYRO_CS_Pin GPIO_PIN_11
#define BMI_GYRO_CS_GPIO_Port GPIOC
#define ENC_RL_A_Pin GPIO_PIN_3
#define ENC_RL_A_GPIO_Port GPIOD
#define ENC_RL_B_Pin GPIO_PIN_4
#define ENC_RL_B_GPIO_Port GPIOD
#define BMI088_NRF_SCK_Pin GPIO_PIN_3
#define BMI088_NRF_SCK_GPIO_Port GPIOB
#define BMI088_NRF_MISO_Pin GPIO_PIN_4
#define BMI088_NRF_MISO_GPIO_Port GPIOB
#define BMI088_NRF_MOSI_Pin GPIO_PIN_5
#define BMI088_NRF_MOSI_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
