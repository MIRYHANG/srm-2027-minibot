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
#include "stm32f1xx_hal.h"

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
#define LED_STATUS1_Pin GPIO_PIN_0
#define LED_STATUS1_GPIO_Port GPIOC
#define LED_STATUS2_Pin GPIO_PIN_1
#define LED_STATUS2_GPIO_Port GPIOC
#define JOY_BTN_Pin GPIO_PIN_2
#define JOY_BTN_GPIO_Port GPIOC
#define JOY_X_Pin GPIO_PIN_0
#define JOY_X_GPIO_Port GPIOA
#define JOY_Y_Pin GPIO_PIN_1
#define JOY_Y_GPIO_Port GPIOA
#define NRF_CSN_Pin GPIO_PIN_4
#define NRF_CSN_GPIO_Port GPIOA
#define NRF_SCK_Pin GPIO_PIN_5
#define NRF_SCK_GPIO_Port GPIOA
#define NRF_MISO_Pin GPIO_PIN_6
#define NRF_MISO_GPIO_Port GPIOA
#define NRF_MOSI_Pin GPIO_PIN_7
#define NRF_MOSI_GPIO_Port GPIOA
#define ARM_DIR_TOGGLE_BTN_Pin GPIO_PIN_0
#define ARM_DIR_TOGGLE_BTN_GPIO_Port GPIOB
#define ARM_STOP_BTN_Pin GPIO_PIN_1
#define ARM_STOP_BTN_GPIO_Port GPIOB
#define ARM_J1_BTN_Pin GPIO_PIN_10
#define ARM_J1_BTN_GPIO_Port GPIOB
#define ARM_J2_BTN_Pin GPIO_PIN_11
#define ARM_J2_BTN_GPIO_Port GPIOB
#define ARM_J3_BTN_Pin GPIO_PIN_12
#define ARM_J3_BTN_GPIO_Port GPIOB
#define ARM_J4_BTN_Pin GPIO_PIN_13
#define ARM_J4_BTN_GPIO_Port GPIOB
#define ARM_J5_BTN_Pin GPIO_PIN_14
#define ARM_J5_BTN_GPIO_Port GPIOB
#define ARM_GRIPPER_BTN_Pin GPIO_PIN_15
#define ARM_GRIPPER_BTN_GPIO_Port GPIOB
#define CHASSIS_CCW_BTN_Pin GPIO_PIN_6
#define CHASSIS_CCW_BTN_GPIO_Port GPIOC
#define CHASSIS_CW_BTN_Pin GPIO_PIN_7
#define CHASSIS_CW_BTN_GPIO_Port GPIOC
#define REMOTE_START_BTN_Pin GPIO_PIN_8
#define REMOTE_START_BTN_GPIO_Port GPIOC
#define REMOTE_ESTOP_BTN_Pin GPIO_PIN_9
#define REMOTE_ESTOP_BTN_GPIO_Port GPIOC
#define NRF_CE_Pin GPIO_PIN_8
#define NRF_CE_GPIO_Port GPIOA
#define NRF_IRQ_Pin GPIO_PIN_8
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_IRQ_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
