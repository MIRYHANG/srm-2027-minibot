//
// Created by YZH on 2026/9/30.
//

#include "arm_pose.h"

#include <stddef.h>

static bool InRange(float value, float min, float max)
{
    return value >= min && value <= max;
}

bool ArmPose_IsValid(const ArmPose_t *pose)
{
    if (pose == NULL)
    {
        return false;
    }

    // 关节角度范围暂按修订后的机械臂手册设置
    if (!InRange(pose->j1_deg, 0.0f, 180.0f)) return false;
    if (!InRange(pose->j2_deg, 30.0f, 90.0f)) return false;
    if (!InRange(pose->j3_deg, 30.0f, 180.0f)) return false;
    if (!InRange(pose->j4_deg, 90.0f, 180.0f)) return false;
    if (!InRange(pose->j5_deg, 0.0f, 180.0f)) return false;

    // 夹爪先用 0～1 表示目标比例，开合方向留待标定
    if (!InRange(pose->gripper_ratio, 0.0f, 1.0f)) return false;

    return true;
}