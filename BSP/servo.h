//
// Created by YZH on 2026/9/29.
//

#ifndef XIAOSAI_SERVO_H
#define XIAOSAI_SERVO_H
#include <stdbool.h>

#include "main.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
} Servo_t;

/**
 * @brief 绑定定时器、通道和脉宽范围
 * @return 参数有效返回 true，否则返回 false
 * @note 适用于当前 TIM8、TIM15 每个计数对应 1 μs 的配置
 */
bool Servo_Init(Servo_t *servo,
                TIM_HandleTypeDef *htim,
                uint32_t channel,
                uint16_t min_pulse_us,
                uint16_t max_pulse_us);

/**
 * @brief 设置舵机控制脉宽
 * @param pulse_us 高电平持续时间，单位 μs
 * @return 设置成功返回 true，参数无效或超出范围返回 false
 * @note 调用前必须成功初始化舵机对象
 */
bool Servo_SetPulseUs(Servo_t *servo, uint16_t pulse_us);

/**
 * @brief 设置初始脉宽并启动 PWM 输出
 * @param initial_pulse_us 启动时的控制脉宽，单位 μs
 * @return 成功返回 HAL_OK，否则返回错误状态
 * @note 舵机对象初始化成功后调用一次
 */
HAL_StatusTypeDef Servo_Start(Servo_t *servo, uint16_t initial_pulse_us);

/**
 * @brief 停止这个舵机通道的 PWM 输出
 * @return 成功返回 HAL_OK，对象无效返回 HAL_ERROR
 * @note 仅停止控制信号，不切断舵机电源
 */
HAL_StatusTypeDef Servo_Stop(Servo_t *servo);

#endif //XIAOSAI_SERVO_H
