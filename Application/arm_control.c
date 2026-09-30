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