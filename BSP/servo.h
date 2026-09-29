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

#endif //XIAOSAI_SERVO_H
