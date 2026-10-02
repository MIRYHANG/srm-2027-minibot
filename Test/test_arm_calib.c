//
// Created by YZH on 2026/10/2.
//
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "arm_calib.h"

int main(void)
{
    uint16_t pulse = 0U;

    assert(ArmCalib_ToPulse(ARM_J2, 30.0f, &pulse));
    assert(pulse == 1000U);

    assert(ArmCalib_ToPulse(ARM_J2, 60.0f, &pulse));
    assert(pulse == 1500U);

    assert(ArmCalib_ToPulse(ARM_J2, 90.0f, &pulse));
    assert(pulse == 2000U);

    // 反向关节的脉宽随目标值增大而减小
    ArmJointCalib_t reverse = {0.0f, 180.0f, 2000U, 1000U, 60.0f};
    assert(ArmCalib_ToPulseWithCalib(&reverse, 0.0f, &pulse));
    assert(pulse == 2000U);
    assert(ArmCalib_ToPulseWithCalib(&reverse, 90.0f, &pulse));
    assert(pulse == 1500U);
    assert(ArmCalib_ToPulseWithCalib(&reverse, 180.0f, &pulse));
    assert(pulse == 1000U);

    // 越界目标应裁剪到关节限位
    float clamped = 0.0f;
    assert(ArmCalib_Clamp(ARM_J2, 0.0f, &clamped));
    assert(clamped == 30.0f);
    assert(ArmCalib_Clamp(ARM_J2, 100.0f, &clamped));
    assert(clamped == 90.0f);
    assert(ArmCalib_ToPulse(ARM_J2, 0.0f, &pulse));
    assert(pulse == 1000U);
    assert(ArmCalib_ToPulse(ARM_J2, 100.0f, &pulse));
    assert(pulse == 2000U);

    // NaN 应被拒绝，失败时不修改输出
    clamped = 42.0f;
    pulse = 1234U;
    assert(!ArmCalib_Clamp(ARM_J2, NAN, &clamped));
    assert(clamped == 42.0f);
    assert(!ArmCalib_ToPulse(ARM_J2, NAN, &pulse));
    assert(pulse == 1234U);

    // 非法索引和空输出指针应被拒绝
    assert(ArmCalib_Get(-1) == NULL);
    assert(ArmCalib_Get(ARM_JOINT_COUNT) == NULL);
    assert(!ArmCalib_Clamp(-1, 60.0f, &clamped));
    assert(clamped == 42.0f);
    assert(!ArmCalib_ToPulse(ARM_JOINT_COUNT, 60.0f, &pulse));
    assert(pulse == 1234U);
    assert(!ArmCalib_Clamp(ARM_J2, 60.0f, NULL));
    assert(!ArmCalib_ToPulse(ARM_J2, 60.0f, NULL));

    puts("Arm calibration tests passed");
    return 0;
}
