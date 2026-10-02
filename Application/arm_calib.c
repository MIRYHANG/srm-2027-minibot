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

bool ArmCalib_ToPulse(int idx, float val, uint16_t *pulse_us)
{
    if (pulse_us == NULL)
    {
        return false;
    }

    const ArmJointCalib_t *calib = ArmCalib_Get(idx);
    if (calib == NULL || calib->min_val >= calib->max_val)
    {
        return false;
    }

    float clamp;
    if (!ArmCalib_Clamp(idx,val,&clamp))
    {
        return false;
    }

    float ratio = (clamp - calib->min_val) / (calib->max_val - calib->min_val);

    float pulse = (float)calib->pulse_at_min_us + ratio * ((float)calib->pulse_at_max_us - (float)calib->pulse_at_min_us);

    *pulse_us = (uint16_t)lroundf(pulse);
    return true;
}

bool ArmCalib_ToPulseWithCalib(const ArmJointCalib_t *calib,
                              float val,
                              uint16_t *pulse_us)
{
    if (calib == NULL || pulse_us == NULL || isnan(val))
    {
        return false;
    }

    if (!isfinite(calib->min_val) ||
        !isfinite(calib->max_val) ||
        calib->min_val >= calib->max_val)
    {
        return false;
    }

    float clamped = val;
    if (clamped < calib->min_val)
    {
        clamped = calib->min_val;
    }
    else if (clamped > calib->max_val)
    {
        clamped = calib->max_val;
    }

    float ratio = (clamped - calib->min_val) / (calib->max_val - calib->min_val);

    float pulse = (float)calib->pulse_at_min_us + ratio * ((float)calib->pulse_at_max_us - (float)calib->pulse_at_min_us);

    *pulse_us = (uint16_t)lroundf(pulse);
    return true;
}