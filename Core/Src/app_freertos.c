/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "tim.h"
#include "encoder.h"
#include "controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static Motor_t motor_fl;
static Motor_t motor_fr;
static Motor_t motor_rl;
static Motor_t motor_rr;

static Encoder_t encode_fl;
static Encoder_t encode_fr;
static Encoder_t encode_rl;
static Encoder_t encode_rr;

static PID_Instance speed_pid_fl;
static PID_Instance speed_pid_fr;
static PID_Instance speed_pid_rl;
static PID_Instance speed_pid_rr;
/* USER CODE END Variables */
osThreadId ChassisControlTHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void Motor_InitAndStart(void);
static void Encoder_InitAndStart(void);
static void SpeedPID_InitAll();
/* USER CODE END FunctionPrototypes */

void StartChassisControlTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ChassisControlT */
  osThreadDef(ChassisControlT, StartChassisControlTask, osPriorityNormal, 0, 256);
  ChassisControlTHandle = osThreadCreate(osThread(ChassisControlT), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartChassisControlTask */
/**
  * @brief  Function implementing the ChassisControlT thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartChassisControlTask */
void StartChassisControlTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisControlTask */
  Motor_InitAndStart();
  Encoder_InitAndStart();

  uint32_t last_wake = osKernelSysTick();
  uint32_t last_sample = last_wake;
  /* Infinite loop */
  for(;;)
  {
    osDelayUntil(&last_wake, 10);

    uint32_t now = osKernelSysTick();
    float dt_s = (float)(now - last_sample) / (float)configTICK_RATE_HZ;
    last_sample = now;

    Encoder_Update(&encode_fl, dt_s);
    Encoder_Update(&encode_fr, dt_s);
    Encoder_Update(&encode_rl, dt_s);
    Encoder_Update(&encode_rr, dt_s);
  }
  /* USER CODE END StartChassisControlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void Motor_InitAndStart(void)
{
  Motor_Init(&motor_fl, &htim1, TIM_CHANNEL_1,
       MOTOR_FL_DIR_GPIO_Port, MOTOR_FL_DIR_Pin, 1);
  Motor_Init(&motor_fr, &htim1, TIM_CHANNEL_2,
             MOTOR_FR_DIR_GPIO_Port, MOTOR_FR_DIR_Pin, 1);
  Motor_Init(&motor_rl, &htim1, TIM_CHANNEL_3,
             MOTOR_RL_DIR_GPIO_Port, MOTOR_RL_DIR_Pin, 1);
  Motor_Init(&motor_rr, &htim1, TIM_CHANNEL_4,
             MOTOR_RR_DIR_GPIO_Port, MOTOR_RR_DIR_Pin, 1);

  if (Motor_Start(&motor_fl) != HAL_OK)
  {
    Error_Handler();
  }

  if (Motor_Start(&motor_fr) != HAL_OK)
  {
    Error_Handler();
  }

  if (Motor_Start(&motor_rl) != HAL_OK)
  {
    Error_Handler();
  }

  if (Motor_Start(&motor_rr) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Encoder_InitAndStart(void)
{
  Encoder_Init(&encode_fl, &htim2,COUNT_PER_REV,DIRECTION_SIGN,SPEED_FILTER_RC_S);
  Encoder_Init(&encode_fr, &htim3,COUNT_PER_REV,DIRECTION_SIGN,SPEED_FILTER_RC_S);
  Encoder_Init(&encode_rl, &htim4,COUNT_PER_REV,DIRECTION_SIGN,SPEED_FILTER_RC_S);
  Encoder_Init(&encode_rr, &htim5,COUNT_PER_REV,DIRECTION_SIGN,SPEED_FILTER_RC_S);

  if (Encoder_Start(&encode_fl) != HAL_OK)
  {
    Error_Handler();
  }
  if (Encoder_Start(&encode_fr) != HAL_OK)
  {
    Error_Handler();
  }
  if (Encoder_Start(&encode_rl) != HAL_OK)
  {
    Error_Handler();
  }
  if (Encoder_Start(&encode_rr) != HAL_OK)
  {
    Error_Handler();
  }
}

static void SpeedPID_InitAll()
{
  PID_Init_Config_s config = {
    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .MaxOut = 1.0f,
    .Improve = PID_IMPROVE_NONE,
  };
  PID_Init(&speed_pid_fl,&config);
  PID_Init(&speed_pid_fr, &config);
  PID_Init(&speed_pid_rl, &config);
  PID_Init(&speed_pid_rr, &config);
}
/* USER CODE END Application */

