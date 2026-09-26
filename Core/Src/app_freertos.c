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
#include "mecanum.h"
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

static float target_rpm_fl = 0.0f;
static float target_rpm_fr = 0.0f;
static float target_rpm_rl = 0.0f;
static float target_rpm_rr = 0.0f;

static MecanumGeometry_t chassis_geometry = {
  .wheel_radius_m = 0.0f,  // TODO：实测轮半径
  .wheelbase_m = 0.0f,     // TODO：前后轮中心距
  .track_width_m = 0.0f,   // TODO：左右轮中心距
};
/* USER CODE END Variables */
osThreadId ChassisControlTHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/*------------------函数封装层-----------------------*/
static void Motor_InitAndStart(void);
static void Encoder_InitAndStart(void);
static void SpeedPID_InitAll(void);
static void ChassisSpeed_Update(float dt_s);
static void WheelSpeed_Update(Encoder_t *encoder, PID_Instance *pid,
                              float target_rpm, float dt_s);
static void Chassis_UpdateTargetRpm(float vx_mps, float vy_mps,
                                    float wz_radps);
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
  * @brief  初始化底盘模块，并周期性更新四轮速度控制。
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartChassisControlTask */
void StartChassisControlTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisControlTask */
  Motor_InitAndStart();
  Encoder_InitAndStart();
  SpeedPID_InitAll();

  uint32_t last_wake = osKernelSysTick();
  uint32_t last_sample = last_wake;
  /* Infinite loop */
  for(;;)
  {
    osDelayUntil(&last_wake, 10);

    uint32_t now = osKernelSysTick();
    float dt_s = (float)(now - last_sample) / (float)configTICK_RATE_HZ;
    last_sample = now;

    Chassis_UpdateTargetRpm(0.0f,0.0f,0.0f);
    ChassisSpeed_Update(dt_s);
  }
  /* USER CODE END StartChassisControlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
 * @brief 绑定四个电机的 PWM 通道和方向引脚，并以零输出启动 PWM。
 */
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

/**
 * @brief 配置并启动四个车轮的编码器定时器。
 * @note 每圈计数和方向修正需用实物验证后，才能将测速结果用于控制。
 */
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

/**
 * @brief 初始化四个车轮的速度 PID 控制器。
 * @note 当前 PID 增益为零，仅作占位，不会驱动电机。
 */
static void SpeedPID_InitAll(void)
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

/**
 * @brief 更新四轮编码器测量值和速度 PID 计算结果。
 * @param dt_s 距离上次更新的实际时间，单位为秒。
 */
static void ChassisSpeed_Update(float dt_s)
{
  if (dt_s <= 0.0f)
  {
    return;
  }

  WheelSpeed_Update(&encode_fl, &speed_pid_fl, target_rpm_fl, dt_s);
  WheelSpeed_Update(&encode_fr, &speed_pid_fr, target_rpm_fr, dt_s);
  WheelSpeed_Update(&encode_rl, &speed_pid_rl, target_rpm_rl, dt_s);
  WheelSpeed_Update(&encode_rr, &speed_pid_rr, target_rpm_rr, dt_s);
}

/**
 * @brief 更新单个车轮的编码器测量值，并计算速度 PID。
 * @param encoder 该车轮的编码器实例。
 * @param pid 该车轮的速度 PID 实例。
 * @param target_rpm 目标车轮转速，单位为 RPM。
 * @param dt_s 距离上次更新的实际时间，单位为秒。
 * @note 当前仅计算 PID，尚未将输出施加到电机。
 */
static void WheelSpeed_Update(Encoder_t *encoder, PID_Instance *pid,
                              float target_rpm, float dt_s)
{
  Encoder_Update(encoder, dt_s);
  (void)PID_Calculate(pid, Encoder_GetSpeedRpm(encoder), target_rpm, dt_s);
}

/**
 * @brief 根据底盘期望速度计算四个车轮的目标转速。
 * @param vx_mps 期望前后速度，正数表示向前，单位为米/秒。
 * @param vy_mps 期望左右速度，正数表示向左，单位为米/秒。
 * @param wz_radps 期望旋转角速度，正数表示逆时针，单位为弧度/秒。
 * @note 计算结果只写入四轮目标 RPM，不直接驱动电机；底盘尺寸无效时目标转速为零。
 */
static void Chassis_UpdateTargetRpm(float vx_mps, float vy_mps,
                                    float wz_radps)
{
  MecanumWheelRpm_t wheels;

  Mecanum_CalculateWheelRpm(&chassis_geometry,
                            vx_mps, vy_mps, wz_radps, &wheels);

  target_rpm_fl = wheels.fl;
  target_rpm_fr = wheels.fr;
  target_rpm_rl = wheels.rl;
  target_rpm_rr = wheels.rr;
}
/* USER CODE END Application */

