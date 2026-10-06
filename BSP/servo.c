//
// Created by YZH on 2026/9/29.
//

/**
 * 该文件代码整体调用关系：
 *      外部接口层：Servo_Init，Servo_Start，Servo_SetPulseUs，Servo_Stop，Servo_SetPosition
 *
 *      内部连接：
 *              Servo_Init：只检查参数并保存配置，不操作硬件；失败时htim为NULL，其余函数靠这一点判断未初始化
 *              Servo_Start：调用SetPulseUs检查对象和脉宽并写入比较值，再启动定时器PWM
 *              Servo_Stop：检查对象已初始化，然后直接关闭定时器PWM
 *              Servo_SetPosition：检查对象和position，把position换算成脉宽，调用SetPulseUs设置PWM
 *
 *      测试时调用：
 *              Init -> Start -> SetPulseUs或SetPosition
 *              可以在测试代码写SetPulseUs或SetPosition均可，只不过数量级不同
 */

#include "servo.h"

/* 与电机、编码器不同，这里不启动定时器，PWM在Servo_Start中才开始输出 */
bool Servo_Init(Servo_t *servo,
                TIM_HandleTypeDef *htim,
                uint32_t channel,
                uint16_t min_pulse_us,
                uint16_t max_pulse_us)
{
    /*----------------------------参数检测---------------------------*/
    if (servo == NULL)
    {
        return false;
    }

    /* 先清零再检查，初始化失败时htim保持NULL，对象处于未绑定状态 */
    *servo = (Servo_t){0};

    if (htim == NULL || min_pulse_us == 0U || min_pulse_us >= max_pulse_us)
    {
        return false;
    }

    if (htim->Instance != TIM8 && htim->Instance != TIM15)
    {
        return false;
    }

    /* 白名单写法：默认通道非法，只有命中允许的通道才置为合法 */
    bool channel_valid = false;

    if (channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2)
    {
        channel_valid = true;
    }

    if (htim->Instance == TIM8)
    {
        if (channel == TIM_CHANNEL_3 || channel == TIM_CHANNEL_4)
        {
            channel_valid = true;
        }
    }

    if (!channel_valid)
    {
        return false;
    }

    /* 每个计数为 1 us，脉宽不能超过完整 PWM 周期 */
    if ((uint32_t)max_pulse_us > htim->Init.Period)
    {
        return false;
    }

    /*----------------------------检测END---------------------------*/

    servo->htim = htim;
    servo->channel = channel;
    servo->min_pulse_us = min_pulse_us;
    servo->max_pulse_us = max_pulse_us;

    return true;
}

bool Servo_SetPulseUs(Servo_t *servo, uint16_t pulse_us)
{
    /*----------------------------参数检测---------------------------*/
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
    /*----------------------------检测END---------------------------*/

    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_us);

    return true;
}

HAL_StatusTypeDef Servo_Start(Servo_t *servo, uint16_t initial_pulse_us)
{
    if (!Servo_SetPulseUs(servo, initial_pulse_us))
    {
        return HAL_ERROR;
    }

    return HAL_TIM_PWM_Start(servo->htim, servo->channel);
}

HAL_StatusTypeDef Servo_Stop(Servo_t *servo)
{
    if (servo == NULL || servo->htim == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_TIM_PWM_Stop(servo->htim, servo->channel);
}

bool Servo_SetPosition(Servo_t *servo, float position)
{
    /*----------------------------参数检测---------------------------*/
    if (servo == NULL || servo->htim == NULL)
    {
        return false;
    }

    if (!(position >= 0.0f && position <= 1.0f))
    {
        return false;
    }
    /*----------------------------检测END---------------------------*/

    float range_us = (float)(servo->max_pulse_us - servo->min_pulse_us);

    /* 截断取整，误差小于1 us */
    uint16_t pulse_us = (uint16_t)(servo->min_pulse_us + position * range_us);

    return Servo_SetPulseUs(servo, pulse_us);
}