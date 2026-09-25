//
// Created by YZH on 2026/9/25.
//

#include "motor.h"

/**
 * @brief 绑定一个电机的 PWM 通道和方向引脚。
 * @param motor 电机实例指针。
 * @param htim PWM 定时器句柄。
 * @param channel PWM 通道，例如 TIM_CHANNEL_1。
 * @param dir_port 方向引脚所在 GPIO 端口。
 * @param dir_pin 方向引脚编号。
 * @param direction_sign 电机方向修正值：+1 或 -1。
 */
void Motor_Init(Motor_t* motor, TIM_HandleTypeDef* htim,
                uint32_t channel,GPIO_TypeDef *dir_port,
                uint16_t dir_pin,int8_t direction_sign)
{
    if (motor == NULL)
    {
        return;
    }

    motor->htim = htim;
    motor->channel = channel;
    motor->dir_port = dir_port;
    motor->dir_pin = dir_pin;
    motor->direction_sign = direction_sign < 0 ? -1 : 1;
}

/**
 * @brief 将 PWM 比较值设为 0，然后启动电机对应的 PWM 通道。
 * @param motor 电机实例指针。
 * @return HAL_OK 表示启动成功；否则返回 HAL 的错误状态。
 */
HAL_StatusTypeDef Motor_Start(Motor_t* motor)
{
    if (motor == NULL || motor->htim == NULL || motor->dir_port == NULL)
    {
        return HAL_ERROR;
    }

    __HAL_TIM_SET_COMPARE(motor->htim,motor->channel,0U);

    return HAL_TIM_PWM_Start(motor->htim,motor->channel);
}

/**
 * @brief 设置电机输出，符号表示方向，绝对值表示 PWM 占空比。
 * @param motor 电机实例指针。
 * @param output 期望输出，约定范围为 -1.0f 到 +1.0f。
 */
void Motor_SetOutput(Motor_t* motor, float output)
{
    if (motor == NULL || motor->htim == NULL || motor->dir_port == NULL)
    {
        return;
    }

    output *= motor->direction_sign;

    if (output > 1.0f)
    {
        output = 1.0f;
    }
    else if (output < -1.0f)
    {
        output = -1.0f;
    }

    if (output == 0.0f)
    {
        Motor_Stop(motor);
        return;
    }

    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0U);

    HAL_GPIO_WritePin(
        motor->dir_port,
        motor->dir_pin,
        output > 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    float duty = output > 0.0f ? output : -output;
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(motor->htim) + 1U;
    uint32_t compare = (uint32_t)(duty * (float)period);

    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, compare);
}

/**
 * @brief 将电机 PWM 输出设为 0，不关闭 PWM 定时器通道。
 * @param motor 电机实例指针。
 */
void Motor_Stop(Motor_t* motor)
{
    if (motor == NULL || motor->htim == NULL)
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(motor->htim,motor->channel,0U);
}