//
// Created by YZH on 2026/10/1.
//

#include "arm_calib.h"

#include <math.h>
#include <stddef.h>

static const ArmJointCalib_t ARM_CALIB[ARM_JOINT_COUNT] =
{
    [ARM_J1]      = {0.0f, 180.0f, 500U, 2500U, 60.0f},  // TODO：实物标定零位和方向
    [ARM_J2]      = {30.0f, 90.0f, 833U, 1500U, 60.0f},  // TODO：实物标定零位和方向
    [ARM_J3]      = {30.0f, 180.0f, 833U, 2500U, 60.0f}, // TODO：实物标定零位和方向
    [ARM_J4]      = {90.0f, 180.0f, 1500U, 2500U, 60.0f}, // TODO：实物标定零位和方向
    [ARM_J5]      = {0.0f, 180.0f, 500U, 2500U, 60.0f},  // TODO：实物标定零位和方向
    [ARM_GRIPPER] = {0.0f, 1.0f, 1000U, 2000U, 2.0f}    // TODO：实物标定零位和方向，并确定夹爪开合位置
};

// 调用者先检查标定项和输入，正负无穷分别裁剪到上下限
static float ClampWithCalib(const ArmJointCalib_t *calib, float in)
{
    if (in < calib->min_val)
    {
        return calib->min_val;
    }
    if (in > calib->max_val)
    {
        return calib->max_val;
    }
    return in;
}

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

    *out = ClampWithCalib(calib, in);

    return true;
}

bool ArmCalib_ToPulse(int idx, float val, uint16_t *pulse_us)
{
    return ArmCalib_ToPulseWithCalib(ArmCalib_Get(idx), val, pulse_us);
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

    float clamped = ClampWithCalib(calib, val);

    float ratio = (clamped - calib->min_val) / (calib->max_val - calib->min_val);

    float pulse = (float)calib->pulse_at_min_us + ratio * ((float)calib->pulse_at_max_us - (float)calib->pulse_at_min_us);

    *pulse_us = (uint16_t)lroundf(pulse);
    return true;
}
