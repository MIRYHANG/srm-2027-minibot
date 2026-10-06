//
// Created by YZH on 2026/10/2.
//
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "arm_calib.h"

// 真实标定值可以变化，只检查每个关节必须满足的约束
static void TestCalibInvariants(void)
{
    for (int idx = 0; idx < ARM_JOINT_COUNT; idx++)
    {
        const ArmJointCalib_t *calib = ArmCalib_Get(idx);
        assert(calib != NULL);
        assert(calib->min_val < calib->max_val);
        assert(calib->pulse_at_min_us >= 500U);
        assert(calib->pulse_at_min_us <= 2500U);
        assert(calib->pulse_at_max_us >= 500U);
        assert(calib->pulse_at_max_us <= 2500U);
        assert(calib->max_speed_per_s > 0.0f);
    }
}

// 使用局部参数验证端点、中点和非中点的插值
static void TestInterpolation(void)
{
    const ArmJointCalib_t calib = {30.0f, 90.0f, 800U, 2200U, 60.0f};
    uint16_t pulse = 0U;
    assert(ArmCalib_ToPulseWithCalib(&calib, 30.0f, &pulse));
    assert(pulse == 800U);
    assert(ArmCalib_ToPulseWithCalib(&calib, 45.0f, &pulse));
    assert(pulse == 1150U);
    assert(ArmCalib_ToPulseWithCalib(&calib, 60.0f, &pulse));
    assert(pulse == 1500U);
    assert(ArmCalib_ToPulseWithCalib(&calib, 90.0f, &pulse));
    assert(pulse == 2200U);

    // 脉宽结果有小数时需要四舍五入
    const ArmJointCalib_t rounding = {0.0f, 4.0f, 1000U, 1003U, 60.0f};
    assert(ArmCalib_ToPulseWithCalib(&rounding, 1.0f, &pulse));
    assert(pulse == 1001U);
    assert(ArmCalib_ToPulseWithCalib(&rounding, 2.0f, &pulse));
    assert(pulse == 1002U);
    assert(ArmCalib_ToPulseWithCalib(&rounding, 3.0f, &pulse));
    assert(pulse == 1002U);
}

// 正向和反向标定都应正确裁剪有限越界值与正负无穷
static void TestClamping(void)
{
    const ArmJointCalib_t calibs[] = {
        {30.0f, 90.0f, 800U, 2200U, 60.0f},
        {30.0f, 90.0f, 2200U, 800U, 60.0f}
    };
    for (size_t idx = 0; idx < sizeof(calibs) / sizeof(calibs[0]); idx++)
    {
        uint16_t pulse = 0U;
        const ArmJointCalib_t *calib = &calibs[idx];
        assert(ArmCalib_ToPulseWithCalib(calib, 0.0f, &pulse));
        assert(pulse == calib->pulse_at_min_us);
        assert(ArmCalib_ToPulseWithCalib(calib, 120.0f, &pulse));
        assert(pulse == calib->pulse_at_max_us);
        assert(ArmCalib_ToPulseWithCalib(calib, -INFINITY, &pulse));
        assert(pulse == calib->pulse_at_min_us);
        assert(ArmCalib_ToPulseWithCalib(calib, INFINITY, &pulse));
        assert(pulse == calib->pulse_at_max_us);
    }
}

static void TestReverse(void)
{
    const ArmJointCalib_t reverse = {30.0f, 90.0f, 2200U, 800U, 60.0f};
    uint16_t pulse = 0U;
    assert(ArmCalib_ToPulseWithCalib(&reverse, 30.0f, &pulse));
    assert(pulse == 2200U);
    assert(ArmCalib_ToPulseWithCalib(&reverse, 45.0f, &pulse));
    assert(pulse == 1850U);
    assert(ArmCalib_ToPulseWithCalib(&reverse, 60.0f, &pulse));
    assert(pulse == 1500U);
    assert(ArmCalib_ToPulseWithCalib(&reverse, 90.0f, &pulse));
    assert(pulse == 800U);
}

static void TestInvalidInput(void)
{
    const ArmJointCalib_t calib = {30.0f, 90.0f, 800U, 2200U, 60.0f};
    // NaN 应被拒绝，失败时不修改输出
    float clamped = 42.0f;
    uint16_t pulse = 1234U;
    assert(!ArmCalib_Clamp(ARM_J1, NAN, &clamped));
    assert(clamped == 42.0f);
    assert(!ArmCalib_ToPulseWithCalib(ArmCalib_Get(ARM_J1), NAN, &pulse));
    assert(pulse == 1234U);
    assert(!ArmCalib_ToPulseWithCalib(&calib, NAN, &pulse));
    assert(pulse == 1234U);

    // 非法索引和空输出指针应被拒绝
    assert(ArmCalib_Get(-1) == NULL);
    assert(ArmCalib_Get(ARM_JOINT_COUNT) == NULL);
    assert(!ArmCalib_Clamp(-1, 60.0f, &clamped));
    assert(!ArmCalib_Clamp(ARM_JOINT_COUNT, 60.0f, &clamped));
    assert(clamped == 42.0f);
    assert(!ArmCalib_ToPulseWithCalib(ArmCalib_Get(-1), 60.0f, &pulse));
    assert(!ArmCalib_ToPulseWithCalib(ArmCalib_Get(ARM_JOINT_COUNT), 60.0f, &pulse));
    assert(pulse == 1234U);
    assert(!ArmCalib_Clamp(ARM_J1, 60.0f, NULL));
    assert(!ArmCalib_ToPulseWithCalib(ArmCalib_Get(ARM_J1), 60.0f, NULL));
    assert(!ArmCalib_ToPulseWithCalib(NULL, 60.0f, &pulse));
    assert(pulse == 1234U);
    assert(!ArmCalib_ToPulseWithCalib(&calib, 60.0f, NULL));
}

int main(void)
{
    TestCalibInvariants();
    TestInterpolation();
    TestClamping();
    TestReverse();
    TestInvalidInput();
    puts("Arm calibration tests passed");
    return 0;
}
