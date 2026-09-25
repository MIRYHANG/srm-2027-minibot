//
// Created by YZH on 2026/9/25.
//

#ifndef XIAOSAI_MOTOR_H
#define XIAOSAI_MOTOR_H
#include "main.h"

typedef struct
{
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    int8_t direction_sign;
} Motor_t;

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
                uint16_t dir_pin,int8_t direction_sign);

/**
 * @brief 将 PWM 比较值设为 0，然后启动电机对应的 PWM 通道。
 * @param motor 电机实例指针。
 * @return HAL_OK 表示启动成功；否则返回 HAL 的错误状态。
 */
HAL_StatusTypeDef Motor_Start(Motor_t* motor);

/**
 * @brief 设置电机输出，符号表示方向，绝对值表示 PWM 占空比。
 * @param motor 电机实例指针。
 * @param output 期望输出，约定范围为 -1.0f 到 +1.0f。
 */
void Motor_SetOutput(Motor_t* motor, float output);

/**
 * @brief 将电机 PWM 输出设为 0，不关闭 PWM 定时器通道。
 * @param motor 电机实例指针。
 */
void Motor_Stop(Motor_t* motor);

#endif //XIAOSAI_MOTOR_H
