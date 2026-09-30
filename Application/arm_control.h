//
// Created by YZH on 2026/9/30.
//

#ifndef XIAOSAI_ARM_CONTROL_H
#define XIAOSAI_ARM_CONTROL_H

#include "arm_pose.h"

typedef struct
{
    ArmPose_t target;
    bool has_target;
} ArmControl_t;

/**
 * @brief 初始化机械臂控制状态
 */
void ArmControl_Init(ArmControl_t *control);

/**
 * @brief 检查并保存新的目标姿态
 * @return 保存成功返回 true，否则返回 false
 * @note 这里只保存目标，不驱动舵机
 */
bool ArmControl_SetTarget(ArmControl_t *control, const ArmPose_t *pose);

/**
 * @brief 读取已保存的目标姿态
 * @param out_pose 用于接收目标姿态
 * @return 有目标返回 true，没有目标返回 false
 */
bool ArmControl_GetTarget(const ArmControl_t *control, ArmPose_t *out_pose);

/**
 * @brief 清除目标姿态
 * @note 只清除缓存，不会停止正在输出的 PWM
 */
void ArmControl_Clear(ArmControl_t *control);

#endif //XIAOSAI_ARM_CONTROL_H
