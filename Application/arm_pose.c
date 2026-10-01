//
// Created by YZH on 2026/9/30.
//

#include "arm_pose.h"

#include <stddef.h>

#include "arm_calib.h"

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

    for (int idx = 0;idx < ARM_JOINT_COUNT;idx++)
    {
        const ArmJointCalib_t *calib = ArmCalib_Get(idx);
        if (calib == NULL || !InRange(pose->joint[idx], calib->min_val, calib->max_val))
        {
            return false;
        }
    }

    return true;
}