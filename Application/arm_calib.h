//
// Created by YZH on 2026/10/1.
//

#ifndef XIAOSAI_ARM_CALIB_H
#define XIAOSAI_ARM_CALIB_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_pose.h"

typedef struct
{
    float min_val;
    float max_val;

    // 两个限位各自对应的脉宽，允许前者大于后者
    uint16_t pulse_at_min_us;
    uint16_t pulse_at_max_us;

    // J1～J5 为度每秒，夹爪为比例每秒
    float max_speed_per_s;
} ArmJointCalib_t;

/**
 * @brief 获取某个关节的只读标定项
 * @return 索引有效时返回标定项地址，否则返回 NULL
 */
const ArmJointCalib_t *ArmCalib_Get(int idx);

/**
 * @brief 将输入值裁剪到关节限位
 * @return 成功返回 true，非法索引、空指针或 NaN 返回 false
 * @note 失败时不修改输出，正负无穷分别裁剪到上下限
 */
bool ArmCalib_Clamp(int idx, float in, float *out);

/**
 * @brief 将关节目标换算为脉宽
 * @note 先裁剪，再线性插值，最后四舍五入
 * @note 失败时不修改输出
 */
bool ArmCalib_ToPulse(int idx, float val, uint16_t *pulse_us);

/**
 * @brief 使用指定标定项换算脉宽
 * @note 与按索引换算共用实现，也用于测试反向标定
 * @note 失败时不修改输出
 */
bool ArmCalib_ToPulseWithCalib(const ArmJointCalib_t *calib,
                              float val,
                              uint16_t *pulse_us);

#endif //XIAOSAI_ARM_CALIB_H
