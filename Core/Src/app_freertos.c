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
#include "remote_uart.h"
#include "remote_phone.h"
#include "servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 暂定超时阈值，后续根据实际发送周期确认
#define PHONE_TIMEOUT_MS 200U

// 限制每周期处理量，避免接收处理一直占用底盘任务
#define PHONE_RX_BUDGET 64U
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

static Servo_t servo_1;
static Servo_t servo_2;
static Servo_t servo_3;
static Servo_t servo_4;
static Servo_t servo_5;
static Servo_t servo_6;

static float target_rpm_fl = 0.0f;
static float target_rpm_fr = 0.0f;
static float target_rpm_rl = 0.0f;
static float target_rpm_rr = 0.0f;

static MecanumGeometry_t chassis_geometry = {
  .wheel_radius_m = 0.0f,  // TODO：实测轮半径
  .wheelbase_m = 0.0f,     // TODO：前后轮中心距
  .track_width_m = 0.0f,   // TODO：左右轮中心距
};

static srm_parser_t phone_parser;       // 保存手机协议解析进度
static RemoteInput_t remote_input;      // 保存两种遥控来源的状态
static RemoteCommand_t remote_command;  // 当前安全遥控命令

// TODO：后续根据底盘能力和调试结果设置
// 当前保持为零，暂不产生运动目标
#define CHASSIS_MAX_VX_MPS   0.0f  // 最大前后速度，m/s
#define CHASSIS_MAX_VY_MPS   0.0f  // 最大横移速度，m/s
#define CHASSIS_MAX_WZ_RADPS 0.0f  // 最大旋转角速度，rad/s
/* USER CODE END Variables */
osThreadId ChassisControlTHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/*------------------函数封装层-----------------------*/
static void Motor_InitAndStart(void);
static void Encoder_InitAndStart(void);
static void SpeedPID_InitAll(void);
static void Servo_InitAll(void);
static void ChassisSpeed_Update(float dt_s);
static void WheelSpeed_Update(Motor_t *motor, Encoder_t *encoder,
                              PID_Instance *pid, float target_rpm, float dt_s);
static void Chassis_UpdateTargetRpm(float vx_mps, float vy_mps,
                                    float wz_radps);
static void PhoneRemote_InitAndStart(void);
static void PhoneRemote_Update(void);
static void Chassis_UpdateFromRemote(const RemoteCommand_t *command);
static void Chassis_Stop(void);

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
  * @brief  初始化底盘模块，并周期性更新四轮速度控制
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
  Servo_InitAll();

  PhoneRemote_InitAndStart();


  uint32_t last_wake = osKernelSysTick();
  uint32_t last_sample = last_wake;
  /* Infinite loop */
  for(;;)
  {
    osDelayUntil(&last_wake, 10);

    uint32_t now = osKernelSysTick();
    float dt_s = (float)(now - last_sample) / (float)configTICK_RATE_HZ;
    last_sample = now;

    PhoneRemote_Update();

    if (!remote_command.enabled || remote_command.stop_requested)
    {
      Chassis_Stop();

      // 停机期间继续采样，保持计数变化与 dt_s 对应
      Encoder_Update(&encode_fl, dt_s);
      Encoder_Update(&encode_fr, dt_s);
      Encoder_Update(&encode_rl, dt_s);
      Encoder_Update(&encode_rr, dt_s);
    }
    else
    {
      Chassis_UpdateFromRemote(&remote_command);
      ChassisSpeed_Update(dt_s);
    }
  }
  /* USER CODE END StartChassisControlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief 将四轮目标和 PWM 输出归零，并清除 PID 动态状态
 */
static void Chassis_Stop(void)
{
  // 清空四轮目标转速
  target_rpm_fl = 0.0f;
  target_rpm_fr = 0.0f;
  target_rpm_rl = 0.0f;
  target_rpm_rr = 0.0f;

  // 将四个电机的 PWM 输出设为零
  Motor_Stop(&motor_fl);
  Motor_Stop(&motor_fr);
  Motor_Stop(&motor_rl);
  Motor_Stop(&motor_rr);

  // 清除积分和历史状态，保留 PID 配置参数
  PID_Clear(&speed_pid_fl);
  PID_Clear(&speed_pid_fr);
  PID_Clear(&speed_pid_rl);
  PID_Clear(&speed_pid_rr);
}

/**
 * @brief 绑定四个电机的 PWM 通道和方向引脚，并以零输出启动 PWM
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
 * @brief 配置并启动四个车轮的编码器定时器
 * @note 每圈计数和方向修正需用实物验证后，才能将测速结果用于控制
 */
static void Encoder_InitAndStart(void)
{
  Encoder_Init(&encode_fl, &htim4,COUNT_PER_REV,DIRECTION_SIGN,SPEED_FILTER_RC_S);
  Encoder_Init(&encode_fr, &htim3,COUNT_PER_REV,DIRECTION_SIGN,SPEED_FILTER_RC_S);
  Encoder_Init(&encode_rl, &htim2,COUNT_PER_REV,DIRECTION_SIGN,SPEED_FILTER_RC_S);
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
 * @brief 初始化四个车轮的速度 PID 控制器
 * @note 当前 PID 增益为零，仅作占位，不会驱动电机
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
 * @brief 将六个舵机对象绑定到对应的定时器通道
 * @note 这里只保存通道和脉宽范围，不启动 PWM
 * @note 1000～2000 μs 是待标定范围，装上机构前需要逐个确认
 */
static void Servo_InitAll(void)
{
  // 舵机 1：PC6，对应 TIM8 通道 1
  if (!Servo_Init(&servo_1, &htim8, TIM_CHANNEL_1, 1000U, 2000U))
  {
    Error_Handler();
  }

  // 舵机 2：PC7，对应 TIM8 通道 2
  if (!Servo_Init(&servo_2, &htim8, TIM_CHANNEL_2, 1000U, 2000U))
  {
    Error_Handler();
  }

  // 舵机 3：PC8，对应 TIM8 通道 3
  if (!Servo_Init(&servo_3, &htim8, TIM_CHANNEL_3, 1000U, 2000U))
  {
    Error_Handler();
  }

  // 舵机 4：PC9，对应 TIM8 通道 4
  if (!Servo_Init(&servo_4, &htim8, TIM_CHANNEL_4, 1000U, 2000U))
  {
    Error_Handler();
  }

  // 舵机 5：PB14，对应 TIM15 通道 1
  if (!Servo_Init(&servo_5, &htim15, TIM_CHANNEL_1, 1000U, 2000U))
  {
    Error_Handler();
  }

  // 舵机 6：PB15，对应 TIM15 通道 2
  if (!Servo_Init(&servo_6, &htim15, TIM_CHANNEL_2, 1000U, 2000U))
  {
    Error_Handler();
  }
}

/**
 * @brief 更新四轮编码器测量值和速度 PID 计算结果
 * @param dt_s 距离上次更新的实际时间，单位为秒
 */
static void ChassisSpeed_Update(float dt_s)
{
  if (dt_s <= 0.0f)
  {
    return;
  }

  WheelSpeed_Update(&motor_fl, &encode_fl, &speed_pid_fl, target_rpm_fl, dt_s);
  WheelSpeed_Update(&motor_fr, &encode_fr, &speed_pid_fr, target_rpm_fr, dt_s);
  WheelSpeed_Update(&motor_rl, &encode_rl, &speed_pid_rl, target_rpm_rl, dt_s);
  WheelSpeed_Update(&motor_rr, &encode_rr, &speed_pid_rr, target_rpm_rr, dt_s);
}

/**
 * @brief 更新单个车轮的编码器测量值，并计算速度 PID
 * @param encoder 该车轮的编码器实例
 * @param pid 该车轮的速度 PID 实例
 * @param target_rpm 目标车轮转速，单位为 RPM
 * @param dt_s 距离上次更新的实际时间，单位为秒
 * @note 当前仅计算 PID，尚未将输出施加到电机
 */
static void WheelSpeed_Update(Motor_t *motor, Encoder_t *encoder,
                              PID_Instance *pid, float target_rpm, float dt_s)
{
  Encoder_Update(encoder, dt_s);
  float output = PID_Calculate(pid, Encoder_GetSpeedRpm(encoder), target_rpm, dt_s);
  Motor_SetOutput(motor, output);
}

/**
 * @brief 根据底盘期望速度计算四个车轮的目标转速
 * @param vx_mps 期望前后速度，正数表示向前，单位为米/秒
 * @param vy_mps 期望左右速度，正数表示向左，单位为米/秒
 * @param wz_radps 期望旋转角速度，正数表示逆时针，单位为弧度/秒
 * @note 计算结果只写入四轮目标 RPM，不直接驱动电机；底盘尺寸无效时目标转速为零
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

/**
 * @brief 初始化手机遥控状态、解析器和接收队列，并启动接收
 * @note 在任务启动阶段调用一次
 */
static void PhoneRemote_InitAndStart(void)
{
  RemoteInput_Init(&remote_input);
  RemoteInput_Select(&remote_input, REMOTE_SOURCE_PHONE);
  srm_parser_init(&phone_parser);

  remote_command = (RemoteCommand_t){0};

  if (!RemoteUart_Init())
  {
    Error_Handler();
  }

  if (!RemoteUart_Start())
  {
    Error_Handler();
  }
}

/**
 * @brief 处理手机接收数据，更新安全遥控命令
 * @note 只在当前底盘任务中调用，不直接驱动电机
 */
static void PhoneRemote_Update(void)
{
  // 默认禁止运动，只有全部检查通过才给出有效命令
  remote_command = (RemoteCommand_t){0};

  if (RemoteUart_HasFault())
  {
    remote_input.phone = (RemoteState_t){0};
    srm_parser_init(&phone_parser);

    if (!RemoteUart_Recover())
    {
      return;
    }

    return;
  }

  RemoteUartByte_t item;

  for (uint32_t count = 0U; count < PHONE_RX_BUDGET; count++)
  {
    // 队列空了或者出现异常，就结束本次读取
    if (!RemoteUart_Read(&item))
    {
      break;
    }

    // 使用字节实际到达的时间，而不是当前处理时间
    RemotePhone_ProcessByte(&phone_parser, &remote_input, item.byte, item.received_ms);
  }

  // 处理过程中也可能出现中断异常，不能使用刚解析的命令
  if (RemoteUart_HasFault())
  {
    remote_input.phone = (RemoteState_t){0};
    srm_parser_init(&phone_parser);
    return;
  }

  // 未收到有效帧、超时、未使能或请求停机时，返回零命令
  remote_command = RemoteInput_GetSafe(&remote_input,
                                       HAL_GetTick(),
                                       PHONE_TIMEOUT_MS);
}

/**
 * @brief 将安全遥控命令换算为底盘速度，再计算四轮目标转速
 * @param command 遥控命令，只读
 * @note 仅更新目标转速，不直接设置电机输出
 */
static void Chassis_UpdateFromRemote(const RemoteCommand_t *command)
{
  if (command == NULL || !command->enabled || command->stop_requested)
  {
    Chassis_UpdateTargetRpm(0.0f, 0.0f, 0.0f);
    return;
  }

  // 遥控量是 -1～1，乘以速度上限得到实际速度
  float vx_mps = command->forward * CHASSIS_MAX_VX_MPS;
  float vy_mps = command->left * CHASSIS_MAX_VY_MPS;
  float wz_radps = command->turn * CHASSIS_MAX_WZ_RADPS;

  // 将底盘速度换算为四个轮子的目标 RPM
  Chassis_UpdateTargetRpm(vx_mps, vy_mps, wz_radps);
}
/* USER CODE END Application */

