//
// Created by YZH on 2026/10/3.
//
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "arm_calib.h"
#include "arm_motion.h"

static ArmPose_t PoseAt(float fraction)
{
    ArmPose_t pose = {0};

    for (int idx = 0; idx < ARM_JOINT_COUNT; idx++)
    {
        const ArmJointCalib_t *calib = ArmCalib_Get(idx);
        assert(calib != NULL);

        pose.joint[idx] = calib->min_val +
                          fraction * (calib->max_val - calib->min_val);
    }

    return pose;
}

static void AssertPoseEqual(const ArmPose_t *actual,
                            const ArmPose_t *expected)
{
    for (int idx = 0; idx < ARM_JOINT_COUNT; idx++)
    {
        assert(actual->joint[idx] == expected->joint[idx]);
    }
}

static void AssertWithinLimits(const ArmPose_t *pose)
{
    for (int idx = 0; idx < ARM_JOINT_COUNT; idx++)
    {
        const ArmJointCalib_t *calib = ArmCalib_Get(idx);
        assert(calib != NULL);
        assert(pose->joint[idx] >= calib->min_val);
        assert(pose->joint[idx] <= calib->max_val);
    }
}

static void TestForwardAndReverse(void)
{
    const ArmJointCalib_t *calib = ArmCalib_Get(ARM_J1);
    assert(calib != NULL);
    assert(calib->max_speed_per_s > 0.0f);

    float range = calib->max_val - calib->min_val;
    float dt_s = range / (4.0f * calib->max_speed_per_s);
    float max_step = calib->max_speed_per_s * dt_s;

    ArmPose_t current = PoseAt(0.0f);
    ArmPose_t target = current;
    ArmPose_t out = {0};

    // 距离大于单步上限：正向只走一步
    target.joint[ARM_J1] = calib->max_val;
    ArmPose_t saved_current = current;
    ArmPose_t saved_target = target;

    assert(ArmMotion_Step(&current, &target, dt_s, &out));
    assert(fabsf(out.joint[ARM_J1] -
                 (calib->min_val + max_step)) < 0.0001f);
    AssertPoseEqual(&current, &saved_current);
    AssertPoseEqual(&target, &saved_target);
    AssertWithinLimits(&out);

    // 距离小于单步上限：直接精确到达
    target.joint[ARM_J1] = calib->min_val + max_step * 0.5f;
    assert(ArmMotion_Step(&current, &target, dt_s, &out));
    assert(out.joint[ARM_J1] == target.joint[ARM_J1]);

    // 反向移动：只走一步
    current.joint[ARM_J1] = calib->max_val;
    target.joint[ARM_J1] = calib->min_val;
    assert(ArmMotion_Step(&current, &target, dt_s, &out));
    assert(fabsf(out.joint[ARM_J1] -
                 (calib->max_val - max_step)) < 0.0001f);
}

static void TestLimits(void)
{
    const ArmJointCalib_t *calib = ArmCalib_Get(ARM_J1);
    assert(calib != NULL);

    float range = calib->max_val - calib->min_val;
    float dt_s = range / (4.0f * calib->max_speed_per_s);
    float max_step = calib->max_speed_per_s * dt_s;

    ArmPose_t current = PoseAt(0.0f);
    ArmPose_t target = current;
    ArmPose_t out = {0};

    // 目标越过上限时，朝上限逼近
    target.joint[ARM_J1] = calib->max_val + range;
    assert(ArmMotion_Step(&current, &target, dt_s, &out));
    assert(fabsf(out.joint[ARM_J1] -
                 (calib->min_val + max_step)) < 0.0001f);
    AssertWithinLimits(&out);

    // 时间足够长时，最终停在上限，而不是越界目标
    assert(ArmMotion_Step(&current, &target, dt_s * 8.0f, &out));
    assert(out.joint[ARM_J1] == calib->max_val);

    // 当前值本身越界时，也先裁剪，再计算下一步
    current.joint[ARM_J1] = calib->min_val - range;
    target.joint[ARM_J1] = calib->max_val;
    assert(ArmMotion_Step(&current, &target, dt_s, &out));
    assert(fabsf(out.joint[ARM_J1] -
                 (calib->min_val + max_step)) < 0.0001f);
    AssertWithinLimits(&out);

    // 下限同样有效
    current.joint[ARM_J1] = calib->max_val;
    target.joint[ARM_J1] = calib->min_val - range;
    assert(ArmMotion_Step(&current, &target, dt_s * 8.0f, &out));
    assert(out.joint[ARM_J1] == calib->min_val);
}

static void TestReachedAndAlias(void)
{
    ArmPose_t current = PoseAt(0.5f);
    ArmPose_t target = current;
    ArmPose_t out = {0};

    assert(ArmMotion_Step(&current, &target, 0.02f, &out));
    AssertPoseEqual(&out, &target);

    current = PoseAt(0.0f);
    target = PoseAt(1.0f);
    ArmPose_t expected = {0};
    ArmPose_t saved_target = target;

    assert(ArmMotion_Step(&current, &target, 0.02f, &expected));
    assert(ArmMotion_Step(&current, &target, 0.02f, &current));
    AssertPoseEqual(&current, &expected);
    AssertPoseEqual(&target, &saved_target);
}

static void AssertRejected(const ArmPose_t *current,
                           const ArmPose_t *target,
                           float dt_s,
                           ArmPose_t *out,
                           const ArmPose_t *saved_out)
{
    assert(!ArmMotion_Step(current, target, dt_s, out));

    if (out != NULL)
    {
        AssertPoseEqual(out, saved_out);
    }
}

static void TestInvalidInput(void)
{
    ArmPose_t current = PoseAt(0.0f);
    ArmPose_t target = PoseAt(1.0f);
    ArmPose_t out = PoseAt(0.5f);
    ArmPose_t saved_out = out;

    AssertRejected(NULL, &target, 0.02f, &out, &saved_out);
    AssertRejected(&current, NULL, 0.02f, &out, &saved_out);
    AssertRejected(&current, &target, 0.02f, NULL, &saved_out);
    AssertRejected(&current, &target, 0.0f, &out, &saved_out);
    AssertRejected(&current, &target, -0.02f, &out, &saved_out);
    AssertRejected(&current, &target, NAN, &out, &saved_out);
    AssertRejected(&current, &target, INFINITY, &out, &saved_out);
    AssertRejected(&current, &target, -INFINITY, &out, &saved_out);

    // 把 NaN 放在最后一个关节，检查前五个计算成功后仍不会部分写入 out
    current.joint[ARM_GRIPPER] = NAN;
    AssertRejected(&current, &target, 0.02f, &out, &saved_out);
    current = PoseAt(0.0f);

    target.joint[ARM_GRIPPER] = NAN;
    AssertRejected(&current, &target, 0.02f, &out, &saved_out);
}

int main(void)
{
    TestForwardAndReverse();
    TestLimits();
    TestReachedAndAlias();
    TestInvalidInput();

    puts("Arm motion tests passed");
    return 0;
}