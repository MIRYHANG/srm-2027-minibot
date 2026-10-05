//
// Created by YZH on 2026/10/5.
//

#include "motor_test.h"

#include <math.h>
#include <stdbool.h>

#include "encoder.h"
#include "iwdg.h"
#include "main.h"
#include "motor.h"
#include "tim.h"

#define MOTOR_COUNT     4
#define TEST_PERIOD_MS  10U
#define TEST_MAX_DUTY   0.3f    // 手动和自动模式都不超过这个占空比

// 1：四个电机按 FL、FR、RL、RR 轮流 正转→停→反转→停，每段 1 s，16 s 一轮
// 0：上电不转，只按 g_motor_duty 输出（调试器暂停时修改）
#define MOTOR_TEST_AUTO 1
#define AUTO_DUTY       0.15f
#define AUTO_STEP_MS    1000U
#define AUTO_PHASES     4U      // 0 正转，1 停，2 反转，3 停

#define LED_BLINK_MS    500U

// 下标顺序统一为 FL、FR、RL、RR，均在调试器中查看
volatile float g_motor_duty[MOTOR_COUNT];       // 当前输出；手动模式下可改
volatile int32_t g_encoder_count[MOTOR_COUNT];  // 编码器累计计数
volatile int32_t g_fwd_delta[MOTOR_COUNT];      // 自动模式：最近一次正转 1 s 内的计数变化
volatile int32_t g_rev_delta[MOTOR_COUNT];      // 自动模式：最近一次反转 1 s 内的计数变化
volatile uint8_t g_driver_fault[MOTOR_COUNT];   // 驱动 FAULT 引脚原始电平，有效电平以驱动手册为准
volatile uint32_t g_auto_step;                  // 自动模式步骤：0～3 FL，4～7 FR，8～11 RL，12～15 RR

static Motor_t motors[MOTOR_COUNT];
static Encoder_t encoders[MOTOR_COUNT];

/**
 * @brief 绑定和正式固件相同的通道；电机方向修正一律为 +1
 */
static void MotorTest_Init(void)
{
    Motor_Init(&motors[0], &htim1, TIM_CHANNEL_1, MOTOR_FL_DIR_GPIO_Port, MOTOR_FL_DIR_Pin, 1);
    Motor_Init(&motors[1], &htim1, TIM_CHANNEL_2, MOTOR_FR_DIR_GPIO_Port, MOTOR_FR_DIR_Pin, 1);
    Motor_Init(&motors[2], &htim1, TIM_CHANNEL_3, MOTOR_RL_DIR_GPIO_Port, MOTOR_RL_DIR_Pin, 1);
    Motor_Init(&motors[3], &htim1, TIM_CHANNEL_4, MOTOR_RR_DIR_GPIO_Port, MOTOR_RR_DIR_Pin, 1);

    Encoder_Init(&encoders[0], &htim4, COUNT_PER_REV, DIRECTION_SIGN, SPEED_FILTER_RC_S);
    Encoder_Init(&encoders[1], &htim3, COUNT_PER_REV, DIRECTION_SIGN, SPEED_FILTER_RC_S);
    Encoder_Init(&encoders[2], &htim2, COUNT_PER_REV, DIRECTION_SIGN, SPEED_FILTER_RC_S);
    Encoder_Init(&encoders[3], &htim5, COUNT_PER_REV, DIRECTION_SIGN, SPEED_FILTER_RC_S);

    for (int idx = 0; idx < MOTOR_COUNT; idx++)
    {
        if (Motor_Start(&motors[idx]) != HAL_OK ||
            Encoder_Start(&encoders[idx]) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

/**
 * @brief 限制占空比；NaN 按 0 处理
 */
static float ClampDuty(float duty)
{
    if (isnan(duty))
    {
        return 0.0f;
    }
    if (duty > TEST_MAX_DUTY)
    {
        return TEST_MAX_DUTY;
    }
    if (duty < -TEST_MAX_DUTY)
    {
        return -TEST_MAX_DUTY;
    }
    return duty;
}

/**
 * @brief 自动模式：按时间算出当前步骤，只让一个电机转，并记录正反转期间的计数变化
 */
static void UpdateAuto(uint32_t elapsed_ms)
{
    static uint32_t last_step = UINT32_MAX;
    static int32_t phase_start_count = 0;

    uint32_t total_steps = (uint32_t)MOTOR_COUNT * AUTO_PHASES;
    uint32_t step = (elapsed_ms / AUTO_STEP_MS) % total_steps;
    uint32_t motor = step / AUTO_PHASES;
    uint32_t phase = step % AUTO_PHASES;

    if (step != last_step)
    {
        // 上一步是正转或反转时，结算这 1 s 内的计数变化
        if (last_step != UINT32_MAX)
        {
            uint32_t last_motor = last_step / AUTO_PHASES;
            uint32_t last_phase = last_step % AUTO_PHASES;
            int32_t delta = g_encoder_count[last_motor] - phase_start_count;

            if (last_phase == 0U)
            {
                g_fwd_delta[last_motor] = delta;
            }
            else if (last_phase == 2U)
            {
                g_rev_delta[last_motor] = delta;
            }
        }

        phase_start_count = g_encoder_count[motor];
        last_step = step;
    }

    float duty = 0.0f;
    if (phase == 0U)
    {
        duty = AUTO_DUTY;
    }
    else if (phase == 2U)
    {
        duty = -AUTO_DUTY;
    }

    for (uint32_t idx = 0U; idx < (uint32_t)MOTOR_COUNT; idx++)
    {
        g_motor_duty[idx] = (idx == motor) ? duty : 0.0f;
    }
    g_auto_step = step;
}

/**
 * @brief 读取四路驱动 FAULT 引脚的原始电平
 */
static void ReadDriverFaults(void)
{
    g_driver_fault[0] = (uint8_t)HAL_GPIO_ReadPin(FL_FAULT_GPIO_Port, FL_FAULT_Pin);
    g_driver_fault[1] = (uint8_t)HAL_GPIO_ReadPin(FR_FAULT_GPIO_Port, FR_FAULT_Pin);
    g_driver_fault[2] = (uint8_t)HAL_GPIO_ReadPin(RL_FAULT_GPIO_Port, RL_FAULT_Pin);
    g_driver_fault[3] = (uint8_t)HAL_GPIO_ReadPin(RR_FAULT_GPIO_Port, RR_FAULT_Pin);
}

/**
 * @brief LED_RUN 每 500 ms 翻转表示程序在跑；任一电机有输出时 LED_LINK 亮
 */
static void UpdateLeds(uint32_t elapsed_ms, bool driving)
{
    GPIO_PinState run = ((elapsed_ms / LED_BLINK_MS) % 2U) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, run);
    HAL_GPIO_WritePin(LED_LINK_GPIO_Port, LED_LINK_Pin, driving ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void MotorTest_Run(void)
{
    MotorTest_Init();

    const float dt_s = (float)TEST_PERIOD_MS / 1000.0f;
    uint32_t start_ms = HAL_GetTick();
    uint32_t next_ms = start_ms;

    for (;;)
    {
        HAL_IWDG_Refresh(&hiwdg);

        uint32_t elapsed_ms = HAL_GetTick() - start_ms;

        if (MOTOR_TEST_AUTO)
        {
            UpdateAuto(elapsed_ms);
        }

        bool driving = false;
        for (int idx = 0; idx < MOTOR_COUNT; idx++)
        {
            float duty = ClampDuty(g_motor_duty[idx]);

            Motor_SetOutput(&motors[idx], duty);
            Encoder_Update(&encoders[idx], dt_s);
            g_encoder_count[idx] = (int32_t)Encoder_GetCount(&encoders[idx]);
            driving = driving || (duty != 0.0f);
        }

        ReadDriverFaults();
        UpdateLeds(elapsed_ms, driving);

        // 按固定节拍等待，不累积每轮的执行时间
        next_ms += TEST_PERIOD_MS;
        while ((int32_t)(HAL_GetTick() - next_ms) < 0)
        {
        }
    }
}
