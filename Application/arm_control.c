//
// Created by YZH on 2026/9/30.
//

#include "arm_control.h"

#include <stddef.h>

void ArmControl_Init(ArmControl_t *control)
{
    if (control == NULL)
    {
        return;
    }

    // 初始时没有可执行的目标姿态
    *control = (ArmControl_t){0};
}

bool ArmControl_SetTarget(ArmControl_t *control, const ArmPose_t *pose)
{
    if (control == NULL || pose == NULL)
    {
        return false;
    }

    if (!ArmPose_IsValid(pose))
    {
        return false;
    }

    control->target = *pose;
    control->has_target = true;
    return true;
}

bool ArmControl_GetTarget(const ArmControl_t *control, ArmPose_t *out_pose)
{
    if (out_pose == NULL)
    {
        return false;
    }

    *out_pose = (ArmPose_t){0};

    if (control == NULL)
    {
        return false;
    }

    if (!control->has_target)
    {
        return false;
    }

    // 将六个目标值复制给调用者
    *out_pose = control->target;
    return true;
}

void ArmControl_Clear(ArmControl_t *control)
{
    if (control == NULL)
    {
        return;
    }

    // 删除旧目标，避免之后误用
    control->target = (ArmPose_t){0};
    control->has_target = false;
}
