//
// Created by YZH on 2026/10/1.
//

#include "arm_calib.h"

#include <math.h>
#include <stddef.h>

/*--------------------定义六个舵机的初始值，严格遵循结构组的数据手册---------------------------*/
static const ArmJointCalib_t ARM_CALIB[ARM_JOINT_COUNT] =
{
    [ARM_J1]      = {0.0f, 180.0f, 500U, 2500U, 60.0f},  // TODO：实物标定零位和方向
    [ARM_J2]      = {30.0f, 90.0f, 833U, 1500U, 60.0f},  // TODO：实物标定零位和方向
    [ARM_J3]      = {30.0f, 180.0f, 833U, 2500U, 60.0f}, // TODO：实物标定零位和方向
    [ARM_J4]      = {90.0f, 180.0f, 1500U, 2500U, 60.0f}, // TODO：实物标定零位和方向
    [ARM_J5]      = {0.0f, 180.0f, 500U, 2500U, 60.0f},  // TODO：实物标定零位和方向
    [ARM_GRIPPER] = {0.0f, 1.0f, 1000U, 2000U, 0.5f}    // TODO：实物标定零位和方向，并确定夹爪开合位置
};

/* ----------------------限幅函数------------------------- */
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

/**
 * @brief 获取某个特定关节，在之后的“*calib=ArmCalib_Get(ARM_xx)”中可以通过calib直接操作对应关节舵机
 * @return 返回标定项地址，否则返回 NULL
 */
const ArmJointCalib_t *ArmCalib_Get(int idx)
{
    if (idx < 0 || idx >= ARM_JOINT_COUNT)
    {
        return NULL;
    }

    return &ARM_CALIB[idx];
}

/**
 * @brief 本质还是限幅函数，摇杆一直推，机械臂到达最大值就不会继续动
 * @return 成功返回 true，非法索引、空指针或 NaN 返回 false
 */
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
