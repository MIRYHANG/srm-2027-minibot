//
// Created by YZH on 2026/9/30.
//

#ifndef XIAOSAI_ARM_POSE_H
#define XIAOSAI_ARM_POSE_H

// 夹爪开合比例：0.0 完全张开，1.0 完全夹紧
#define ARM_GRIPPER_OPEN   0.0f
#define ARM_GRIPPER_CLOSED 1.0f

#include <stdbool.h>

typedef enum
{
    ARM_J1 = 0,
    ARM_J2,
    ARM_J3,
    ARM_J4,
    ARM_J5,
    ARM_GRIPPER,

    ARM_JOINT_COUNT = 6
} ArmJointIndex_t;

typedef struct
{
    // J1～J5 的单位为度，夹爪为 0～1 的比例，见 ARM_GRIPPER_OPEN / ARM_GRIPPER_CLOSED
    float joint[ARM_JOINT_COUNT];
} ArmPose_t;

/**
 * @brief 检查全部关节目标是否在标定范围内
 * @return 全部有效返回 true，否则返回 false
 * @note 这里只校验，不裁剪目标
 */
bool ArmPose_IsValid(const ArmPose_t *pose);

#endif
