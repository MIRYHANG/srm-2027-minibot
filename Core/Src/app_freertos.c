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
#include "arm_cycle.h"
#include "iwdg.h"
#include "safety.h"
#include "queue.h"
#include "task_monitor.h"
#include "i2c.h"
#include "i2c_bus_hal.h"
#include "ina226.h"
#include "ssd1306.h"
#include "power_display.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 暂定超时阈值，后续根据实际发送周期确认
#define PHONE_TIMEOUT_MS 200U

// 限制每周期处理字节数，避免协议任务长时间占用 CPU
#define PHONE_RX_BUDGET 64U
#define REMOTE_PUBLISH_STALE_MS 30U

// INA226 功率采样
#define POWER_SHUNT_OHM     0.002f // 分流电阻 2 mΩ，量程约 ±40.96 A
#define POWER_INA226_ADDR   0x40U  // TODO：按 A0/A1 接法确认；找不到时自动扫描 0x40～0x4F
#define POWER_RETRY_PERIODS 10U    // 初始化失败后每 10 个周期（约 1 s）重试一次

// OLED 功率显示
#define OLED_ADDR            SSD1306_ADDR_LOW // TODO：按模块 SA0 接法确认，少数模块是 0x3D
#define OLED_POWER_UP_MS     100U // 上电后等电源稳定再初始化
#define OLED_REFRESH_PERIODS 5U   // 每 5 个周期（约 500 ms）刷新一次
#define OLED_RETRY_PERIODS   10U  // 初始化失败后每 10 个周期（约 1 s）重试一次

typedef struct
{
  RemoteCommand_t command;
  uint32_t stamp_ms; // 协议任务发布这份安全命令的时间
} RemotePublished_t;
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

// 舵机通道按关节顺序排列：J1～J5、夹爪
// TODO：和机械组确认 servo_1～6 与关节的对应关系
static Servo_t *const arm_servos[ARM_JOINT_COUNT] = {
  &servo_1, &servo_2, &servo_3, &servo_4, &servo_5, &servo_6
};

// 只由 ArmTask 访问
static ArmCycle_t arm_cycle;

static float target_rpm_fl = 0.0f;
static float target_rpm_fr = 0.0f;
static float target_rpm_rl = 0.0f;
static float target_rpm_rr = 0.0f;

static MecanumGeometry_t chassis_geometry = {
  .wheel_radius_m = 0.0f,  // TODO：实测轮半径
  .wheelbase_m = 0.0f,     // TODO：前后轮中心距
  .track_width_m = 0.0f,   // TODO：左右轮中心距
};

// 以下四个状态只由 ProtocolTask 访问
// 其他任务通过 remote_cmd_queue 获取命令副本
static srm_parser_t phone_parser;
static RemoteInput_t remote_input;
static RemoteCommand_t remote_command;
static RemotePhoneArm_t phone_arm;

// 长度为 1，只保存协议任务最近发布的一份完整命令
static QueueHandle_t remote_cmd_queue = NULL;

// 以下状态只由 SensorTask 访问；power_reading 可在调试器里直接查看
static I2cBus_t i2c2_bus;
static Ina226_t power_sensor;
static Ina226Reading_t power_reading;      // 最近一次有效读数
static bool power_reading_valid = false;   // 本周期读数是否有效
static uint8_t power_sensor_addr = POWER_INA226_ADDR;
static uint32_t power_retry_countdown = 0U;

// 以下状态只由 SensorTask 访问；oled 含 1 KB 显存，必须是静态变量
static Ssd1306_t oled;
static PowerDisplayLine_t oled_lines[POWER_DISPLAY_LINES];
static uint32_t oled_refresh_countdown = 0U;
static uint32_t oled_retry_countdown = 0U;
static ResetCause_t reset_cause = RESET_CAUSE_UNKNOWN;

// TODO：后续根据底盘能力和调试结果设置
// 当前保持为零，暂不产生运动目标
#define CHASSIS_MAX_VX_MPS   0.0f  // 最大前后速度，m/s
#define CHASSIS_MAX_VY_MPS   0.0f  // 最大横移速度，m/s
#define CHASSIS_MAX_WZ_RADPS 0.0f  // 最大旋转角速度，rad/s
/* USER CODE END Variables */
osThreadId ChassisControlTHandle;
osThreadId ArmTaskHandle;
osThreadId ProtocolTaskHandle;
osThreadId SensorTaskHandle;

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
static bool Remote_GetLatest(RemoteCommand_t *out);
static void ArmServos_Start(const uint16_t pulse_us[]);
static void ArmServos_Apply(const uint16_t pulse_us[]);
static bool PowerSensor_Start(void);
static void PowerSensor_Update(void);
static ResetCause_t ResetCause_FromFlags(uint32_t flags);
static void Oled_Update(void);

/* USER CODE END FunctionPrototypes */

void StartChassisControlTask(void const * argument);
void StartArmTask(void const * argument);
void StartProtocolTask(void const * argument);
void StartSensorTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
   (void)xTask;
   (void)pcTaskName;
   taskDISABLE_INTERRUPTS();
   Safety_EmergencyStop();
   for (;;)
   {
   }
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
   taskDISABLE_INTERRUPTS();
   Safety_EmergencyStop();
   for (;;)
   {
   }
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  Servo_InitAll();
  TaskMonitor_Init(HAL_GetTick());
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
  remote_cmd_queue = xQueueCreate(1, sizeof(RemotePublished_t));
  if (remote_cmd_queue == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ChassisControlT */
  osThreadDef(ChassisControlT, StartChassisControlTask, osPriorityHigh, 0, 256);
  ChassisControlTHandle = osThreadCreate(osThread(ChassisControlT), NULL);

  /* definition and creation of ArmTask */
  osThreadDef(ArmTask, StartArmTask, osPriorityNormal, 0, 256);
  ArmTaskHandle = osThreadCreate(osThread(ArmTask), NULL);

  /* definition and creation of ProtocolTask */
  osThreadDef(ProtocolTask, StartProtocolTask, osPriorityAboveNormal, 0, 256);
  ProtocolTaskHandle = osThreadCreate(osThread(ProtocolTask), NULL);

  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, StartSensorTask, osPriorityLow, 0, 384);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

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
  (void)argument;

  Motor_InitAndStart();
  Encoder_InitAndStart();
  SpeedPID_InitAll();

  uint32_t last_wake = osKernelSysTick();
  uint32_t last_sample = last_wake;

  for (;;)
  {
    osDelayUntil(&last_wake, 10U);

    // 底盘任务能执行到这里，并且其他任务均正常时才喂狗
    bool all_alive = TaskMonitor_AllAlive(HAL_GetTick());

    if (all_alive)
    {
      HAL_IWDG_Refresh(&hiwdg);
    }

    uint32_t now = osKernelSysTick();
    float dt_s = (float)(now - last_sample) /
                 (float)configTICK_RATE_HZ;
    last_sample = now;

    RemoteCommand_t command;
    (void)Remote_GetLatest(&command);

    if (!all_alive || !command.enabled || command.stop_requested)
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
      Chassis_UpdateFromRemote(&command);
      ChassisSpeed_Update(dt_s);
    }
  }
  /* USER CODE END StartChassisControlTask */
}

/* USER CODE BEGIN Header_StartArmTask */
/**
* @brief Function implementing the ArmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartArmTask */
void StartArmTask(void const * argument)
{
  /* USER CODE BEGIN StartArmTask */
  /* Infinite loop */
  (void)argument;

  // 初始化阶段的阻塞时间不能超过本任务 100 ms 的心跳期限
  // 需要等待时使用 osDelay，不使用 HAL_Delay
  // osDelay 不会自动打心跳，等待期间仍须遵守心跳期限

  // 先按 STOW 算好 6 路脉宽再启动 PWM，舵机收到的第一个脉宽就是 STOW
  if (!ArmCycle_Init(&arm_cycle))
  {
    Error_Handler();
  }
  ArmServos_Start(arm_cycle.pulse_us);

  uint32_t last_wake = osKernelSysTick();
  uint32_t last_sample = last_wake;

  for (;;)
  {
    osDelayUntil(&last_wake, 20U);

    uint32_t now = osKernelSysTick();
    float dt_s = (float)(now - last_sample) /
                 (float)configTICK_RATE_HZ;
    last_sample = now;

    // 命令过期时得到全零命令，enabled 为 false，机械臂当场停住
    RemoteCommand_t command;
    (void)Remote_GetLatest(&command);

    // 失败时保持上一次的脉宽：不停 PWM，也不输出 0，舵机继续出力
    if (ArmCycle_Step(&arm_cycle, &command, dt_s))
    {
      ArmServos_Apply(arm_cycle.pulse_us);
    }

    // 计算失败不等于任务卡死，照常打心跳
    TaskMonitor_Beat(TASK_ID_ARM, HAL_GetTick());
  }
  /* USER CODE END StartArmTask */
}

/* USER CODE BEGIN Header_StartProtocolTask */
/**
* @brief Function implementing the ProtocolTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProtocolTask */
void StartProtocolTask(void const * argument)
{
  /* USER CODE BEGIN StartProtocolTask */
  /* Infinite loop */
  (void)argument;

  // 初始化必须在协议任务的启动宽限内完成
  PhoneRemote_InitAndStart();

  uint32_t last_wake = osKernelSysTick();

  for (;;)
  {
    // 串口异常时，此函数仍会把 remote_command 保持为全零
    PhoneRemote_Update();

    RemotePublished_t published = {
      .command = remote_command,
      .stamp_ms = HAL_GetTick()
  };

    // 长度为 1 的队列始终覆盖为最新命令，不积累历史命令
    if (xQueueOverwrite(remote_cmd_queue, &published) != pdPASS)
    {
      Error_Handler();
    }

    // 发布完成后打心跳，串口故障本身不等于协议任务卡死
    TaskMonitor_Beat(TASK_ID_PROTOCOL, HAL_GetTick());

    osDelayUntil(&last_wake, 5U);
  }
  /* USER CODE END StartProtocolTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Infinite loop */
  (void)argument;

  // 初始化阶段的阻塞时间不能超过本任务 400 ms 的心跳期限
  // 需要等待时使用 osDelay，不使用 HAL_Delay
  // osDelay 不会自动打心跳，等待期间仍须遵守心跳期限

  // I2C2 由 OLED 和 INA226 共用，两者都只在本任务中访问，不需要互斥锁
  if (!I2cBusHal_Init(&i2c2_bus, &hi2c2))
  {
    Error_Handler();
  }

  reset_cause = ResetCause_FromFlags(g_reset_flags);

  // 等 OLED 电源稳定；加上首次全屏刷新（400 kHz 下约 30 ms，100 kHz 下约 110 ms），
  // 第一次打心跳前最多约 220 ms，小于 400 ms 期限
  osDelay(OLED_POWER_UP_MS);

  uint32_t last_wake = osKernelSysTick();

  for (;;)
  {
    // 传感器故障不影响行驶，只是读数无效
    PowerSensor_Update();

    Oled_Update();

    TaskMonitor_Beat(TASK_ID_SENSOR, HAL_GetTick());

    osDelayUntil(&last_wake, 100U);
  }
  /* USER CODE END StartSensorTask */
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
 * @note 500～2500 μs 为底层脉宽保护范围，关节限位由 ARM_CALIB 决定
 */
static void Servo_InitAll(void)
{
  // 舵机 1：PC6，对应 TIM8 通道 1
  if (!Servo_Init(&servo_1, &htim8, TIM_CHANNEL_1, 500U, 2500U))
  {
    Error_Handler();
  }

  // 舵机 2：PC7，对应 TIM8 通道 2
  if (!Servo_Init(&servo_2, &htim8, TIM_CHANNEL_2, 500U, 2500U))
  {
    Error_Handler();
  }

  // 舵机 3：PC8，对应 TIM8 通道 3
  if (!Servo_Init(&servo_3, &htim8, TIM_CHANNEL_3, 500U, 2500U))
  {
    Error_Handler();
  }

  // 舵机 4：PC9，对应 TIM8 通道 4
  if (!Servo_Init(&servo_4, &htim8, TIM_CHANNEL_4, 500U, 2500U))
  {
    Error_Handler();
  }

  // 舵机 5：PB14，对应 TIM15 通道 1
  if (!Servo_Init(&servo_5, &htim15, TIM_CHANNEL_1, 500U, 2500U))
  {
    Error_Handler();
  }

  // 舵机 6：PB15，对应 TIM15 通道 2
  if (!Servo_Init(&servo_6, &htim15, TIM_CHANNEL_2, 500U, 2500U))
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
  RemotePhoneArm_Init(&phone_arm); // selected = ARM_J1，也就是 0
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
    phone_arm.last_dpad = 0U;
    srm_parser_init(&phone_parser);

    (void)RemoteUart_Recover();
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
    RemotePhone_ProcessByte(&phone_parser, &remote_input, &phone_arm,
                            item.byte, item.received_ms);
  }

  // 处理过程中也可能出现中断异常，不能使用刚解析的命令
  if (RemoteUart_HasFault())
  {
    remote_input.phone = (RemoteState_t){0};
    phone_arm.last_dpad = 0U;
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

/**
 * @brief 读取最近发布的安全命令，不从队列中取走数据
 * @return 成功且未过期返回 true，否则输出全零并返回 false
 * @note 底盘和机械臂均可调用，两者不会互相消费队列中的命令
 */
static bool Remote_GetLatest(RemoteCommand_t *out)
{
  if (out == NULL)
  {
    return false;
  }

  *out = (RemoteCommand_t){0};

  RemotePublished_t published;

  if (remote_cmd_queue == NULL ||
      xQueuePeek(remote_cmd_queue, &published, 0U) != pdPASS)
  {
    return false;
  }

  if ((uint32_t)(HAL_GetTick() - published.stamp_ms) >
      REMOTE_PUBLISH_STALE_MS)
  {
    return false;
  }

  *out = published.command;
  return true;
}

/**
 * @brief 设置 6 路初始脉宽并启动舵机 PWM
 * @param pulse_us 按关节顺序排列的脉宽
 * @note 启动失败说明定时器配置有误，进入 Error_Handler
 */
static void ArmServos_Start(const uint16_t pulse_us[])
{
  for (int idx = 0; idx < ARM_JOINT_COUNT; idx++)
  {
    if (Servo_Start(arm_servos[idx], pulse_us[idx]) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

/**
 * @brief 把 6 路脉宽写入舵机
 * @param pulse_us 按关节顺序排列的脉宽
 * @note 脉宽来自 ARM_CALIB，都在 Servo_InitAll 的 500～2500 μs 内；
 *       某一路被拒绝时该路保持上一次的脉宽
 */
static void ArmServos_Apply(const uint16_t pulse_us[])
{
  for (int idx = 0; idx < ARM_JOINT_COUNT; idx++)
  {
    (void)Servo_SetPulseUs(arm_servos[idx], pulse_us[idx]);
  }
}

/**
 * @brief 查找 INA226 地址并初始化
 * @return 成功返回 true
 * @note 总线卡死时最多 16 次 I2C 超时，约 160 ms，小于本任务 400 ms 的心跳期限
 */
static bool PowerSensor_Start(void)
{
  uint8_t addr = 0U;

  if (!Ina226_FindAddress(&i2c2_bus, power_sensor_addr, &addr))
  {
    return false;
  }

  // 记住找到的地址，下次重试先试它
  power_sensor_addr = addr;
  return Ina226_Init(&power_sensor, &i2c2_bus, addr, POWER_SHUNT_OHM);
}

/**
 * @brief 每周期读取一次功率；芯片未就绪时按间隔重试初始化
 */
static void PowerSensor_Update(void)
{
  if (!power_sensor.ready)
  {
    power_reading_valid = false;

    // 不每个周期都重试，避免总线卡死时反复等超时
    if (power_retry_countdown > 0U)
    {
      power_retry_countdown--;
      return;
    }

    if (!PowerSensor_Start())
    {
      power_retry_countdown = POWER_RETRY_PERIODS;
      return;
    }
  }

  Ina226Reading_t reading;
  power_reading_valid = Ina226_Read(&power_sensor, &reading);

  if (power_reading_valid)
  {
    power_reading = reading;
  }
}

/**
 * @brief 把上电时的复位标志换算成复位原因
 * @note 上电时 BOR 和 PIN 标志会同时置位，所以先判断 BOR
 */
static ResetCause_t ResetCause_FromFlags(uint32_t flags)
{
  if ((flags & RCC_CSR_IWDGRSTF) != 0U)
  {
    return RESET_CAUSE_IWDG;
  }
  if ((flags & RCC_CSR_WWDGRSTF) != 0U)
  {
    return RESET_CAUSE_WWDG;
  }
  if ((flags & RCC_CSR_LPWRRSTF) != 0U)
  {
    return RESET_CAUSE_LOW_POWER;
  }
  if ((flags & RCC_CSR_SFTRSTF) != 0U)
  {
    return RESET_CAUSE_SOFTWARE;
  }
  if ((flags & RCC_CSR_BORRSTF) != 0U)
  {
    return RESET_CAUSE_POWER;
  }
  if ((flags & RCC_CSR_PINRSTF) != 0U)
  {
    return RESET_CAUSE_PIN;
  }
  return RESET_CAUSE_UNKNOWN;
}

/**
 * @brief 每 500 ms 刷新一次 OLED；屏幕未就绪时按间隔重试初始化
 * @note 只发送内容有变化的页，平时每次刷新只有电压、电流、功率三行
 */
static void Oled_Update(void)
{
  if (!oled.ready)
  {
    if (oled_retry_countdown > 0U)
    {
      oled_retry_countdown--;
      return;
    }

    if (!Ssd1306_Init(&oled, &i2c2_bus, OLED_ADDR))
    {
      oled_retry_countdown = OLED_RETRY_PERIODS;
      return;
    }

    // 初始化成功后立刻显示
    oled_refresh_countdown = 0U;
  }

  if (oled_refresh_countdown > 0U)
  {
    oled_refresh_countdown--;
    return;
  }
  oled_refresh_countdown = OLED_REFRESH_PERIODS - 1U;

  PowerDisplay_Format(&power_reading, power_reading_valid, reset_cause, oled_lines);

  for (uint8_t page = 0U; page < POWER_DISPLAY_LINES; page++)
  {
    (void)Ssd1306_WriteLine(&oled, page, oled_lines[page]);
  }

  // 发送失败时驱动把 ready 清零，下个周期重新初始化
  (void)Ssd1306_Flush(&oled);
}
/* USER CODE END Application */

