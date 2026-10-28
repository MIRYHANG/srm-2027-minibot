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
#include "queue.h"
#include "task_monitor.h"
#include "i2c.h"
#include "i2c_bus_hal.h"
#include "ina226.h"
#include "ssd1306.h"
#include "power_display.h"
#include "robot_cmd.h"
#include "safety.h"
#include "safety_input.h"
#include "chassis_drive.h"
#include "NRF24.h"
#include "nrf_link.h"
#include "handheld_packet.h"
#include "remote_handheld.h"
#include "spi1_bus.h"
#include "spi.h"
#include "bmi088.h"
#include "imu_yaw.h"
#include "heading_hold.h"
#include "battery_monitor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 当前使用的遥控来源：REMOTE_SOURCE_PHONE 手机蓝牙，REMOTE_SOURCE_HANDHELD 自制手柄
#define REMOTE_ACTIVE_SOURCE REMOTE_SOURCE_PHONE

// 暂定超时阈值，后续根据实际发送周期确认
#define PHONE_TIMEOUT_MS 200U
#define HANDHELD_TIMEOUT_MS 100U // 手柄每 10 ms 发一包，连续丢约 10 包判定掉线

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

// 急停和驱动故障输入，引脚已在 CubeMX 中配置为上拉输入
// 急停按下时 PE10 接地，读到低电平；未接急停或松开时由上拉读到高电平
// 注意：急停线断开时读到高电平，不会触发急停
#define ESTOP_ACTIVE_LEVEL          GPIO_PIN_RESET
// TODO：按驱动芯片确认。nFAULT 通常为低电平有效的开漏输出
#define MOTOR_FAULT_ACTIVE_LEVEL    GPIO_PIN_RESET

// 状态灯
#define LED_ON_LEVEL           GPIO_PIN_SET   // TODO：按 LED 接法确认
#define LED_OFF_LEVEL          GPIO_PIN_RESET
#define LED_RUN_TOGGLE_PERIODS 100U // ProtocolTask 5 ms * 100，LED_RUN 每 500 ms 翻转一次

typedef struct
{
  RobotCmdOutput_t output; // 底盘和机械臂命令
  uint32_t stamp_ms;       // 协议任务发布这份命令的时间
} RobotPublished_t;

#define HANDHELD_RADIO_POWERUP_MS 100U // 模块上电后需要稳定的时间
#define HANDHELD_RADIO_RETRY_MS   100U // 配置失败后的重试间隔
#define HANDHELD_RADIO_RX_BUDGET  3U   // 每周期最多读几包，等于 RX FIFO 深度
#define HANDHELD_RADIO_CHECK_MS   100U // 每 100 ms 回读一次模块配置
#define HANDHELD_SPI_LOCK_TIMEOUT_MS 2U // SPI1 与 BMI088 共用，拿不到锁就跳过本周期，靠手柄超时兜底

// BMI088 航向
#define IMU_PERIOD_MS            5U    // 200 Hz 采样
#define IMU_RETRY_MS             1000U // 初始化失败后的重试间隔；初始化要占用 SPI1 约 50 ms，不宜太频繁
#define IMU_INIT_LOCK_TIMEOUT_MS 5U
#define IMU_READ_LOCK_TIMEOUT_MS 1U    // 拿不到锁就跳过本次采样，不算错误
#define IMU_ERROR_LIMIT          10U   // 连续读失败这么多次，重新初始化
#define IMU_STALE_MS             20U   // 底盘读到的航向超过这么久没更新，视为无效
// TODO：按 BMI088 在车上的安装方向确认。芯片平放、Z 轴朝上时，逆时针转动 Z 轴角速度为正
#define IMU_YAW_AXIS             2U    // 0=X，1=Y，2=Z
#define IMU_YAW_SIGN             1.0f  // 逆时针转动时读数为负就改成 -1.0f
#define IMU_RAD_TO_DEG           57.29577951f

// 电池监测
#define BATTERY_UPDATE_DT_S 0.1f // SensorTask 周期 100 ms

typedef struct
{
  float yaw_deg;     // 航向，逆时针为正
  float rate_dps;    // 角速度，逆时针为正
  bool valid;        // 已完成零偏标定
  uint32_t stamp_ms; // ImuTask 发布的时间
} ImuPublished_t;
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

// 以下状态只由 ProtocolTask 访问
// 其他任务通过 robot_cmd_queue 获取命令副本
static srm_parser_t phone_parser;
static RemoteInput_t remote_input;
static RemotePhoneArm_t phone_arm;
static RobotCmd_t robot_cmd;
static SafetyInput_t safety_input;
static uint32_t led_run_count = 0U;
static bool radio_ready;
static uint32_t radio_last_try_ms;
static uint32_t radio_check_ms;

// 手柄链路测试用，在调试器 Live Watch 里观察
volatile uint32_t radio_rx_count;  // 收到的全部载荷
volatile uint32_t radio_bad_count; // 其中校验失败的载荷

// 手机链路计数，只由 ProtocolTask 修改
static uint32_t phone_rx_bytes;
static uint32_t phone_rx_frames;

// ProtocolTask 写，SensorTask 读，在临界区内整体复制
static PowerDisplayRemote_t remote_diag;

// ArmTask 写，SensorTask 读；16 位读写是原子的，不需要加锁
static volatile uint16_t arm_pulse_diag[ARM_JOINT_COUNT];

// 长度为 1，只保存协议任务最近发布的一份完整命令
static QueueHandle_t robot_cmd_queue = NULL;

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

// SensorTask 写，ProtocolTask 读；枚举按 32 位整数一次写入，不需要加锁
static BatteryMonitor_t battery_monitor;
static volatile BatteryLevel_t battery_level = BATTERY_UNKNOWN;

// 以下状态只由 ImuTask 访问；imu_yaw 可在调试器里直接查看航向和零偏
static Bmi088_t bmi088;
static ImuYaw_t imu_yaw;

// 长度为 1，只保存 ImuTask 最近发布的航向
static QueueHandle_t imu_queue = NULL;

// 只由底盘任务访问
static HeadingHold_t heading_hold;

osThreadId ImuTaskHandle;

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
static void PhoneRemote_InitAndStart(void);
static void PhoneRemote_Update(void);
static void HandheldRadio_Update(void);
static void HandheldRadio_PublishStatus(const RobotCmd_t *robot, uint8_t safety_active);
static uint32_t RemoteTimeoutMs(RemoteSource_t source);
static void Chassis_UpdateFromCmd(const ChassisCmd_t *cmd);
static void Chassis_Stop(void);
static bool RobotCmd_GetLatest(RobotCmdOutput_t *out);
static uint8_t Safety_ReadRaw(void);
static void StatusLed_Update(const RobotCmdOutput_t *output, uint8_t safety_active);
static void ArmServos_Start(const uint16_t pulse_us[]);
static void ArmServos_Apply(const uint16_t pulse_us[]);
static bool PowerSensor_Start(void);
static void PowerSensor_Update(void);
static ResetCause_t ResetCause_FromFlags(uint32_t flags);
static void Oled_Update(void);
static void Bmi088_DelayMs(uint32_t ms);
static bool ImuSensor_Start(void);
static bool ImuSensor_ReadYawRate(float *raw_dps, bool *skipped);
static bool Imu_GetLatest(ImuPublished_t *out);
void StartImuTask(void const * argument);

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
  // SPI1 由 BMI088（ImuTask）和 NRF24（ProtocolTask）共用
  if (!Spi1Bus_Init())
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  robot_cmd_queue = xQueueCreate(1, sizeof(RobotPublished_t));
  if (robot_cmd_queue == NULL)
  {
    Error_Handler();
  }

  imu_queue = xQueueCreate(1, sizeof(ImuPublished_t));
  if (imu_queue == NULL)
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
  // 放在 USER CODE 区，CubeMX 重新生成时不会被删掉
  osThreadDef(ImuTask, StartImuTask, osPriorityNormal, 0, 256);
  ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);
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

    RobotCmdOutput_t command;
    (void)RobotCmd_GetLatest(&command);

    if (!all_alive || !command.chassis.enabled)
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
      // 平移且没按自转键时，用 IMU 航向修正 turn，防止跑偏
      ChassisCmd_t chassis = command.chassis;
      ImuPublished_t imu;
      bool imu_valid = Imu_GetLatest(&imu);
      chassis.turn = HeadingHold_Update(&heading_hold, &chassis, imu_valid,
                                        imu.yaw_deg, imu.rate_dps, dt_s);

      Chassis_UpdateFromCmd(&chassis);
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
    RobotCmdOutput_t command;
    (void)RobotCmd_GetLatest(&command);

    // 失败时保持上一次的脉宽：不停 PWM，也不输出 0，舵机继续出力
    if (ArmCycle_Step(&arm_cycle, &command.arm, dt_s))
    {
      ArmServos_Apply(arm_cycle.pulse_us);
    }

    for (int idx = 0; idx < ARM_JOINT_COUNT; idx++)
    {
      arm_pulse_diag[idx] = arm_cycle.pulse_us[idx];
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
  SafetyInput_Init(&safety_input);
  RobotCmd_Init(&robot_cmd);

  uint32_t last_wake = osKernelSysTick();

  for (;;)
  {
    // 接收并解析手机数据，有效帧存进 remote_input
    PhoneRemote_Update();

    // 锁加在调用处：HandheldRadio_Update 有多处提前 return，加在函数里容易漏解锁
    if (Spi1Bus_Lock(HANDHELD_SPI_LOCK_TIMEOUT_MS))
    {
      HandheldRadio_Update();
      Spi1Bus_Unlock();
    }

    RemoteCommand_t latest;
    bool online = RemoteInput_GetLatest(&remote_input, HAL_GetTick(),
                                        RemoteTimeoutMs(remote_input.selected), &latest);

    uint8_t safety_active = SafetyInput_Update(&safety_input,Safety_ReadRaw());

    RobotPublished_t published =
    {
      .stamp_ms = HAL_GetTick()
    };
    RobotCmd_Update(&robot_cmd, online, &latest, safety_active, &published.output);
    StatusLed_Update(&published.output, safety_active);

    int arm_joint = RemotePhoneArm_GetSelected(&phone_arm);
    if (arm_joint < 0)
    {
      arm_joint = ARM_J1;
    }

    PowerDisplayRemote_t diag = {
      .rx_bytes = phone_rx_bytes,
      .rx_frames = phone_rx_frames,
      .online = online,
      .s1 = latest.enabled,
      .s2 = latest.stop_requested,
      .safety = safety_active,
      .enabled = robot_cmd.enabled,
      .arm_joint = (uint8_t)(arm_joint + 1),
      .arm_jog = published.output.arm.motion.jog[arm_joint],
      .arm_enabled = published.output.arm.enabled
    };
    taskENTER_CRITICAL();
    remote_diag = diag;
    taskEXIT_CRITICAL();
    if (Spi1Bus_Lock(HANDHELD_SPI_LOCK_TIMEOUT_MS))
    {
      HandheldRadio_PublishStatus(&robot_cmd, safety_active);
      Spi1Bus_Unlock();
    }
    
    // 长度为 1 的队列始终覆盖为最新命令，不积累历史命令
    if (xQueueOverwrite(robot_cmd_queue, &published) != pdPASS)
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

    // 欠压时由 ProtocolTask 经 SafetyInput 让车失能
    battery_level = BatteryMonitor_Update(&battery_monitor, power_reading.bus_v,
                                          power_reading_valid, BATTERY_UPDATE_DT_S);

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

  // 下次开动时重新锁定当前航向
  HeadingHold_Reset(&heading_hold);
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
 * @brief 初始化手机遥控状态、解析器和接收队列，并启动接收
 * @note 在任务启动阶段调用一次
 */
static void PhoneRemote_InitAndStart(void)
{
  RemoteInput_Init(&remote_input);
  RemotePhoneArm_Init(&phone_arm); // selected = ARM_J1，也就是 0
  RemoteInput_Select(&remote_input, REMOTE_ACTIVE_SOURCE);
  srm_parser_init(&phone_parser);

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
 * @brief 接收并解析手机数据，有效帧存进 remote_input
 * @note 只在 ProtocolTask 中调用；能不能动由 RobotCmd 判断
 * @note 串口异常时清空手机状态，相当于掉线：停止输出，但不改变使能状态
 */
static void PhoneRemote_Update(void)
{
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

    phone_rx_bytes++;

    // 使用字节实际到达的时间，而不是当前处理时间
    if (RemotePhone_ProcessByte(&phone_parser, &remote_input, &phone_arm,
                                item.byte, item.received_ms))
    {
      phone_rx_frames++;
    }
  }

  // 处理过程中也可能出现中断异常，不能使用刚解析的命令
  if (RemoteUart_HasFault())
  {
    remote_input.phone = (RemoteState_t){0};
    phone_arm.last_dpad = 0U;
    srm_parser_init(&phone_parser);
    return;
  }

}

/**
 * @brief 按当前挡位把底盘命令换算成四轮目标转速
 * @param cmd 底盘命令，只读
 * @note 仅更新目标转速，不直接设置电机输出
 * @note 推满摇杆时最快那个轮子等于挡位转速；未使能或命令非法时目标转速为零
 * @note 目标转速不经过斜坡，摇杆回中时目标立即为零，由速度 PID 尽快刹停
 */
static void Chassis_UpdateFromCmd(const ChassisCmd_t *cmd)
{
  MecanumWheelRpm_t wheels;

  (void)ChassisDrive_ToWheelRpm(cmd, &wheels);

  target_rpm_fl = wheels.fl;
  target_rpm_fr = wheels.fr;
  target_rpm_rl = wheels.rl;
  target_rpm_rr = wheels.rr;
}

/**
 * @brief 读取最近发布的安全命令，不从队列中取走数据
 * @return 成功且未过期返回 true，否则输出全零并返回 false
 * @note 底盘和机械臂均可调用，两者不会互相消费队列中的命令
 */
static bool RobotCmd_GetLatest(RobotCmdOutput_t *out)
{
  if (out == NULL)
  {
    return false;
  }

  *out = (RobotCmdOutput_t){0};

  RobotPublished_t published;

  if (robot_cmd_queue == NULL ||
      xQueuePeek(robot_cmd_queue, &published, 0U) != pdPASS)
  {
    return false;
  }

  if ((uint32_t)(HAL_GetTick() - published.stamp_ms) >
      REMOTE_PUBLISH_STALE_MS)
  {
    return false;
  }

  *out = published.output;
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
  PowerDisplay_FormatBattery(battery_level, BatteryMonitor_Percent(battery_monitor.filtered_v),
                             oled_lines[5]);

  PowerDisplayRemote_t diag;
  taskENTER_CRITICAL();
  diag = remote_diag;
  taskEXIT_CRITICAL();
  PowerDisplay_FormatRemote(&diag, oled_lines[1]);

  // 标题行换成机械臂诊断
  uint16_t pulse_us = 0U;
  if (diag.arm_joint >= 1U && diag.arm_joint <= ARM_JOINT_COUNT)
  {
    pulse_us = arm_pulse_diag[diag.arm_joint - 1U];
  }
  PowerDisplay_FormatArm(&diag, pulse_us, oled_lines[0]);

  for (uint8_t page = 0U; page < POWER_DISPLAY_LINES; page++)
  {
    (void)Ssd1306_WriteLine(&oled, page, oled_lines[page]);
  }

  // 发送失败时驱动把 ready 清零，下个周期重新初始化
  (void)Ssd1306_Flush(&oled);
}


/**
 * @brief 读取急停和 4 路驱动故障引脚，换算成 SafetyInput 的有效位
 */
static uint8_t Safety_ReadRaw(void)
{
  uint8_t raw = 0U;

  if (HAL_GPIO_ReadPin(FL_FAULT_GPIO_Port, FL_FAULT_Pin) == MOTOR_FAULT_ACTIVE_LEVEL)
  {
    raw |= SAFETY_FAULT_FL;
  }
  if (HAL_GPIO_ReadPin(FR_FAULT_GPIO_Port, FR_FAULT_Pin) == MOTOR_FAULT_ACTIVE_LEVEL)
  {
    raw |= SAFETY_FAULT_FR;
  }
  if (HAL_GPIO_ReadPin(RL_FAULT_GPIO_Port, RL_FAULT_Pin) == MOTOR_FAULT_ACTIVE_LEVEL)
  {
    raw |= SAFETY_FAULT_RL;
  }
  if (HAL_GPIO_ReadPin(RR_FAULT_GPIO_Port, RR_FAULT_Pin) == MOTOR_FAULT_ACTIVE_LEVEL)
  {
    raw |= SAFETY_FAULT_RR;
  }
  if (HAL_GPIO_ReadPin(ESTOPN_GPIO_Port, ESTOPN_Pin) == ESTOP_ACTIVE_LEVEL)
  {
    raw |= SAFETY_ESTOP;
  }
  if (battery_level == BATTERY_CRITICAL)
  {
    raw |= SAFETY_UNDERVOLT;
  }

  return raw;
}

/**
 * @brief 更新 3 个状态灯
 * @note LED_RUN 闪烁表示调度正常；LED_LINK 亮表示底盘允许运动；LED_FAULT 亮表示急停或驱动故障
 */
static void StatusLed_Update(const RobotCmdOutput_t *output, uint8_t safety_active)
{
  led_run_count++;
  if (led_run_count >= LED_RUN_TOGGLE_PERIODS)
  {
    led_run_count = 0U;
    HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
  }

  HAL_GPIO_WritePin(LED_LINK_GPIO_Port, LED_LINK_Pin,
                    output->chassis.enabled ? LED_ON_LEVEL : LED_OFF_LEVEL);
  HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin,
                    safety_active != 0U ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

/**
 * @brief 当前遥控来源允许的最长无有效帧时间
 */
static uint32_t RemoteTimeoutMs(RemoteSource_t source)
{
  return source == REMOTE_SOURCE_HANDHELD ? HANDHELD_TIMEOUT_MS : PHONE_TIMEOUT_MS;
}

/**
 * @brief 接收手柄数据，有效包存进 remote_input.handheld
 * @note 只在 ProtocolTask 中调用；不阻塞，配置失败后每 100 ms 重试一次
 * @note 协议任务 30 ms 内必须打心跳，所以不能在这里 osDelay 等模块上电
 * @note 模块异常时清空手柄状态，相当于掉线：停止输出，但不改变使能状态
 */
static void HandheldRadio_Update(void)
{
  uint32_t now = HAL_GetTick();

  if (!radio_ready)
  {
    if (now < HANDHELD_RADIO_POWERUP_MS ||
        (uint32_t)(now - radio_last_try_ms) < HANDHELD_RADIO_RETRY_MS)
    {
      return;
    }
    radio_last_try_ms = now;
    radio_ready = NrfLink_Configure(nRF24_MODE_RX);
    return;
  }

  uint8_t buf[NRF_LINK_MAX_PAYLOAD];
  uint8_t len = 0U;
  for (uint32_t count = 0U; count < HANDHELD_RADIO_RX_BUDGET; count++)
  {
    if (!NrfLink_Receive(buf, &len))
    {
      break;
    }
    radio_rx_count++;

    // 使用读出这一包的时间；每周期都会读空 FIFO，误差不超过一个任务周期
    if (!RemoteHandheld_ProcessPayload(&remote_input, buf, len, HAL_GetTick()))
    {
      radio_bad_count++;
    }
  }

  // SPI 出错或模块单独复位丢了配置时，丢弃手柄状态，下一周期重新配置模块
  bool lost = NRF24_Port_HasFault();
  if (!lost && (uint32_t)(now - radio_check_ms) >= HANDHELD_RADIO_CHECK_MS)
  {
    radio_check_ms = now;
    lost = !NrfLink_IsConfigured();
  }

  if (lost)
  {
    remote_input.handheld = (RemoteState_t){0};
    radio_ready = false;
  }
}

/**
 * @brief 把小车状态预装成 ACK 载荷，手柄下次发包时随应答带回
 * @note 在 RobotCmd_Update 之后调用，回传的是本周期的使能和挡位
 */
static void HandheldRadio_PublishStatus(const RobotCmd_t *robot, uint8_t safety_active)
{
  if (!radio_ready)
  {
    return;
  }

  HhStatus_t status = {
    .flags = (uint8_t)((robot->enabled ? HH_STATUS_ENABLED : 0U) |
                       (safety_active != 0U ? HH_STATUS_SAFETY : 0U)),
    .gear = robot->gear
  };

  uint8_t buf[HH_STATUS_LEN];
  if (HhStatus_Encode(&status, buf, sizeof(buf)) == HH_STATUS_LEN)
  {
    (void)NrfLink_QueueAck(buf, (uint8_t)sizeof(buf));
  }
}
/**
 * @brief 给 BMI088 驱动用的延时
 * @note osDelay(n) 实际等待 n-1～n 个 tick，多等 1 个 tick 保证不短于 ms
 */
static void Bmi088_DelayMs(uint32_t ms)
{
  osDelay(ms + 1U);
}

/**
 * @brief 初始化 BMI088，成功后重新开始零偏标定
 * @note 初始化期间占用 SPI1 约 50 ms，手柄接收会跳过这几个周期
 */
static bool ImuSensor_Start(void)
{
  const Bmi088Config_t config = {
    .hspi = &hspi1,
    .acc_cs_port = BMI_ACC_CS_GPIO_Port,
    .acc_cs_pin = BMI_ACC_CS_Pin,
    .gyro_cs_port = BMI_GYRO_CS_GPIO_Port,
    .gyro_cs_pin = BMI_GYRO_CS_Pin,
    .delay_ms = Bmi088_DelayMs,
    .acc_range = BMI088_ACC_RANGE_6G,
    .gyro_range = BMI088_GYRO_RANGE_1000DPS,
  };

  if (!Spi1Bus_Lock(IMU_INIT_LOCK_TIMEOUT_MS))
  {
    return false;
  }
  Bmi088Status_t status = Bmi088_Init(&bmi088, &config);
  Spi1Bus_Unlock();

  ImuYaw_Init(&imu_yaw);
  return status == BMI088_OK;
}

/**
 * @brief 读一次竖直轴角速度，已换算成 °/s、逆时针为正，未减零偏
 * @param skipped 拿不到 SPI1 锁时置 true，此时返回 false 但不算读取失败
 */
static bool ImuSensor_ReadYawRate(float *raw_dps, bool *skipped)
{
  *skipped = false;

  if (!Spi1Bus_Lock(IMU_READ_LOCK_TIMEOUT_MS))
  {
    *skipped = true;
    return false;
  }
  Bmi088Sample_t sample;
  Bmi088Status_t status = Bmi088_ReadSample(&bmi088, &sample);
  Spi1Bus_Unlock();

  if (status != BMI088_OK)
  {
    return false;
  }

  *raw_dps = sample.gyro[IMU_YAW_AXIS] * IMU_YAW_SIGN * IMU_RAD_TO_DEG;
  return true;
}

/**
 * @brief 读取最近发布的航向，不从队列中取走
 * @return 有数据、已完成标定且未过期返回 true；否则输出全零并返回 false
 */
static bool Imu_GetLatest(ImuPublished_t *out)
{
  *out = (ImuPublished_t){0};

  ImuPublished_t published;
  if (imu_queue == NULL || xQueuePeek(imu_queue, &published, 0U) != pdPASS)
  {
    return false;
  }

  if (!published.valid ||
      (uint32_t)(HAL_GetTick() - published.stamp_ms) > IMU_STALE_MS)
  {
    return false;
  }

  *out = published;
  return true;
}

/**
 * @brief 每 5 ms 读一次 BMI088 角速度，积分得到航向并发布
 * @note 上电后车须静止约 2 s 完成零偏标定，标定完成前航向无效，底盘不修正
 * @note IMU 故障不影响行驶，只是不做航向修正
 */
void StartImuTask(void const * argument)
{
  (void)argument;

  ImuYaw_Init(&imu_yaw);

  uint32_t last_wake = osKernelSysTick();
  uint32_t last_sample = last_wake;
  uint32_t last_try_ms = HAL_GetTick() - IMU_RETRY_MS; // 第一次立刻尝试
  uint32_t error_count = 0U;

  for (;;)
  {
    osDelayUntil(&last_wake, IMU_PERIOD_MS);

    if (!bmi088.ready)
    {
      uint32_t now_ms = HAL_GetTick();
      if ((uint32_t)(now_ms - last_try_ms) >= IMU_RETRY_MS)
      {
        last_try_ms = now_ms;
        error_count = 0U;
        (void)ImuSensor_Start();
        last_sample = osKernelSysTick();
      }

      ImuPublished_t invalid = {.valid = false, .stamp_ms = HAL_GetTick()};
      (void)xQueueOverwrite(imu_queue, &invalid);
    }
    else
    {
      float raw_dps = 0.0f;
      bool skipped = false;

      if (ImuSensor_ReadYawRate(&raw_dps, &skipped))
      {
        // 用两次成功采样之间的时间积分，跳过的周期不会丢掉转过的角度
        uint32_t now = osKernelSysTick();
        float dt_s = (float)(now - last_sample) / (float)configTICK_RATE_HZ;
        last_sample = now;
        error_count = 0U;

        ImuPublished_t published = {
          .valid = ImuYaw_Update(&imu_yaw, raw_dps, dt_s),
          .yaw_deg = imu_yaw.yaw_deg,
          .rate_dps = imu_yaw.rate_dps,
          .stamp_ms = HAL_GetTick()
        };
        (void)xQueueOverwrite(imu_queue, &published);
      }
      else if (!skipped)
      {
        error_count++;
        if (error_count >= IMU_ERROR_LIMIT)
        {
          bmi088.ready = false;
        }
      }
    }

    TaskMonitor_Beat(TASK_ID_IMU, HAL_GetTick());
  }
}
/* USER CODE END Application */

