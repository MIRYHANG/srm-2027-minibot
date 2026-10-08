//
// Created by YZH on 2026/10/7.
//

#ifndef XIAOSAI_SAFETY_INPUT_H
#define XIAOSAI_SAFETY_INPUT_H

#define SAFETY_FAULT_FL 0x01U // 左前驱动故障
#define SAFETY_FAULT_FR 0x02U // 右前驱动故障
#define SAFETY_FAULT_RL 0x04U // 左后驱动故障
#define SAFETY_FAULT_RR 0x08U // 右后驱动故障
#define SAFETY_ESTOP    0x10U // 急停按下
#define SAFETY_ALL_MASK 0x1FU
#define SAFETY_INPUT_COUNT 5U

#define SAFETY_RELEASE_SAMPLES 20U

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint8_t active;                            // 去抖后的有效输入
    uint8_t release_count[SAFETY_INPUT_COUNT]; // 各输入连续无效的次数
} SafetyInput_t;

/**
 * @brief 清空状态，所有输入视为无效
 * @note 上电也视为一次停止，需要先关闭一次使能才允许运动
 */
void SafetyInput_Init(SafetyInput_t *state);

/**
 * @brief 输入一次采样，返回去抖后的有效输入
 * @param raw 本次采样的有效输入，SAFETY_ALL_MASK 以外的位被忽略
 * @return 去抖后的有效输入，非 0 表示必须停止运动；state 为 NULL 时返回 SAFETY_ESTOP
 * @note 有效立刻生效；要连续 SAFETY_RELEASE_SAMPLES 次无效才解除，抖动只会让车多停一会
 */
uint8_t SafetyInput_Update(SafetyInput_t *state, uint8_t raw);

#endif //XIAOSAI_SAFETY_INPUT_H
