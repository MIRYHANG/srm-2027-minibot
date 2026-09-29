//
// Created by YZH on 2026/9/29.
//

#include "servo.h"

bool Servo_Init(Servo_t *servo,
                TIM_HandleTypeDef *htim,
                uint32_t channel,
                uint16_t min_pulse_us,
                uint16_t max_pulse_us)
{
    if (servo == NULL)
    {
        return false;
    }

    // 初始化失败时，对象保持未绑定状态
    *servo = (Servo_t){0};

    // 检查定时器和脉宽范围
    if (htim == NULL || min_pulse_us == 0U || min_pulse_us >= max_pulse_us)
    {
        return false;
    }

    // 当前舵机使用 TIM8 或 TIM15
    if (htim->Instance != TIM8 && htim->Instance != TIM15)
    {
        return false;
    }

    // 默认认为通道不合法
    bool channel_valid = false;

    // TIM8 和 TIM15 都允许使用通道 1、2
    if (channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2)
    {
        channel_valid = true;
    }

    // 通道 3、4 只允许用于 TIM8
    if (htim->Instance == TIM8)
    {
        if (channel == TIM_CHANNEL_3 || channel == TIM_CHANNEL_4)
        {
            channel_valid = true;
        }
    };

    if (!channel_valid)
    {
        return false;
    }

    // 每个计数为 1 μs，脉宽必须小于完整 PWM 周期
    if ((uint32_t)max_pulse_us > htim->Init.Period)
    {
        return false;
    }

    // 将配置保存到这个舵机对象
    servo->htim = htim;
    servo->channel = channel;
    servo->min_pulse_us = min_pulse_us;
    servo->max_pulse_us = max_pulse_us;

    return true;
}

bool Servo_SetPulseUs(Servo_t *servo, uint16_t pulse_us)
{
    if (servo == NULL || servo->htim == NULL)
    {
        return false;
    }

    if (pulse_us > servo->max_pulse_us || pulse_us < servo->min_pulse_us)
    {
        return false;
    }

    if ((uint32_t)pulse_us > __HAL_TIM_GET_AUTORELOAD(servo->htim))
    {
        return false;
    }

    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_us);

    return true;
}