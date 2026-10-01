//
// Created by YZH on 2026/10/1.
//

#include "arm_calib.h"

#include <math.h>
#include <stddef.h>

static const ArmJointCalib_t ARM_CALIB[ARM_JOINT_COUNT] =
{
    [ARM_J1]      = {0.0f,180.0f, 1000U, 2000U, 60.0f },
    [ARM_J2]      = {30.0f,90.0f, 1000U, 2000U, 60.0f },
    [ARM_J3]      = {30.0f,180.0f, 1000U, 2000U, 60.0f },
    [ARM_J4]      = {90.0f,180.0f, 1000U, 2000U, 60.0f },
    [ARM_J5]      = {0.0f,180.0f, 1000U, 2000U, 60.0f },
    [ARM_GRIPPER] = {0.0f,1.0f, 1000U, 2000U, 60.0f / 180.0f }
};

const ArmJointCalib_t *ArmCalib_Get(int idx)
{
    if (idx < 0 || idx >= ARM_JOINT_COUNT)
    {
        return NULL;
    }

    return &ARM_CALIB[idx];
}

bool ArmCalib_Clamp(int idx, float in, float *out)
{
    const ArmJointCalib_t *calib = ArmCalib_Get(idx);
    if (calib == NULL || out == NULL || isnan(in))
    {
        return false;
    }

    if (in < calib->min_val)
    {
        *out = calib->min_val;
    }
    else if (in > calib->max_val)
    {
        *out = calib->max_val;
    }
    else
    {
        *out = in;
    }

    return true;
}