//
// Created by YZH on 2026/9/30.
//

#ifndef XIAOSAI_ARM_POSE_H
#define XIAOSAI_ARM_POSE_H
#include <stdbool.h>

typedef struct
{
    float j1_deg;
    float j2_deg;
    float j3_deg;
    float j4_deg;
    float j5_deg;
    float gripper_ratio;
} ArmPose_t;

/**
 * @brief 检查机械臂目标是否在设计范围内
 * @param pose 待检查的目标姿态
 * @return 全部有效返回 true，否则返回 false
 */
bool ArmPose_IsValid(const ArmPose_t *pose);

#endif //XIAOSAI_ARM_POSE_H
