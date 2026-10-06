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

    /*----------用于机械臂限位，与结构组给出的手册限位一致，防止机械臂姿态扭曲-------*/
    uint16_t pulse_at_min_us;
    uint16_t pulse_at_max_us;

    // J1～J5 为度每秒，夹爪为比例每秒
    float max_speed_per_s;
} ArmJointCalib_t;

/**
 * @brief 获取某个特定关节，在之后的“*calib=ArmCalib_Get(ARM_xx)”中可以通过calib直接操作对应关节舵机
 * @return 返回标定项地址，否则返回 NULL
 */
const ArmJointCalib_t *ArmCalib_Get(int idx);

/**
 * @brief 本质还是限幅函数，摇杆一直推，机械臂到达最大值就不会继续动
 * @return 成功返回 true，非法索引、空指针或 NaN 返回 false
 */
bool ArmCalib_Clamp(int idx, float in, float *out);

/**
 * @brief 使用指定标定项，将关节目标值换算为脉宽
 * @note 先裁剪(也就是ArmCalib_Clamp限幅)，再插值，最后四舍五入
 * @note 按关节换算时传入 ArmCalib_Get(idx)；索引非法时得到 NULL，本函数返回 false
 * @note 失败时不修改输出
 */
bool ArmCalib_ToPulseWithCalib(const ArmJointCalib_t *calib,
                              float val,
                              uint16_t *pulse_us);

#endif //XIAOSAI_ARM_CALIB_H
