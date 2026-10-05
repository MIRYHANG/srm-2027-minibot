//
// Created by YZH on 2026/10/5.
//
//
// Created by YZH on 2026/10/5.
//
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "arm_calib.h"
#include "arm_control.h"

#define DT_S      0.02f
#define TOLERANCE 0.0001f

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

static void AssertNear(float actual, float expected)
{
    assert(fabsf(actual - expected) < TOLERANCE);
}

static float MaxStep(int idx)
{
    const ArmJointCalib_t *calib = ArmCalib_Get(idx);
    assert(calib != NULL);
    return calib->max_speed_per_s * DT_S;
}

/* 以 current 初始化目标，得到一个 has_target 为 true 的控制状态 */
static ArmControl_t ControlAt(const ArmPose_t *current)
{
    ArmControl_t control;
    ArmRemoteCommand_t zero = {0};

    ArmControl_Init(&control);
    assert(ArmControl_Update(&control, current, &zero, DT_S));
    assert(control.has_target);
    return control;
}

/* 非法调用必须返回 false，且 control 逐字节不变 */
static void AssertRejected(ArmControl_t *control, const ArmPose_t *current,
                           const ArmRemoteCommand_t *cmd, float dt_s)
{
    ArmControl_t saved;
    memcpy(&saved, control, sizeof(saved));

    assert(!ArmControl_Update(control, current, cmd, dt_s));
    assert(memcmp(&saved, control, sizeof(saved)) == 0);
}

static void TestPresets(void)
{
    static const ArmPreset_t valid[] = {ARM_PRESET_GRAB_READY, ARM_PRESET_STOW};

    for (size_t idx = 0; idx < sizeof(valid) / sizeof(valid[0]); idx++)
    {
        ArmPose_t pose = {0};
        assert(ArmControl_GetPreset(valid[idx], &pose));
        assert(ArmPose_IsValid(&pose));
    }

    static const ArmPreset_t invalid[] = {
        ARM_PRESET_NONE,
        (ArmPreset_t)(ARM_PRESET_STOW + 1),
        (ArmPreset_t)99
    };

    for (size_t idx = 0; idx < sizeof(invalid) / sizeof(invalid[0]); idx++)
    {
        ArmPose_t pose = PoseAt(0.25f);
        ArmPose_t saved = pose;

        assert(!ArmControl_GetPreset(invalid[idx], &pose));
        AssertPoseEqual(&pose, &saved);
    }

    assert(!ArmControl_GetPreset(ARM_PRESET_STOW, NULL));
}

static void TestFirstUpdateAdoptsCurrent(void)
{
    ArmRemoteCommand_t zero = {0};
    ArmControl_t control;
    ArmPose_t target;

    // 当前位姿在限位内：目标等于当前位姿
    ArmPose_t current = PoseAt(0.5f);
    ArmControl_Init(&control);
    assert(ArmControl_Update(&control, &current, &zero, DT_S));
    assert(ArmControl_GetTarget(&control, &target));
    AssertPoseEqual(&target, &current);

    // 当前位姿越界：目标是裁剪后的值
    const ArmJointCalib_t *j2 = ArmCalib_Get(ARM_J2);
    const ArmJointCalib_t *j3 = ArmCalib_Get(ARM_J3);
    assert(j2 != NULL && j3 != NULL);

    current.joint[ARM_J2] = j2->min_val - 10.0f;
    current.joint[ARM_J3] = j3->max_val + 10.0f;
    ArmControl_Init(&control);
    assert(ArmControl_Update(&control, &current, &zero, DT_S));
    assert(control.target.joint[ARM_J2] == j2->min_val);
    assert(control.target.joint[ARM_J3] == j3->max_val);
    assert(control.target.joint[ARM_J1] == current.joint[ARM_J1]);
}

static void TestJogStep(void)
{
    static const float jogs[] = {1.0f, -1.0f, 0.5f, -0.5f};

    for (int joint = 0; joint < ARM_GRIPPER; joint++)
    {
        for (size_t j = 0; j < sizeof(jogs) / sizeof(jogs[0]); j++)
        {
            ArmPose_t current = PoseAt(0.5f);
            ArmControl_t control = ControlAt(&current);
            ArmRemoteCommand_t cmd = {0};
            cmd.jog[joint] = jogs[j];

            assert(ArmControl_Update(&control, &current, &cmd, DT_S));

            for (int idx = 0; idx < ARM_JOINT_COUNT; idx++)
            {
                if (idx == joint)
                {
                    AssertNear(control.target.joint[idx],
                               current.joint[idx] + jogs[j] * MaxStep(idx));
                }
                else
                {
                    assert(control.target.joint[idx] == current.joint[idx]);
                }
            }
        }
    }
}

static void TestJogClampedAtLimits(void)
{
    for (int joint = 0; joint < ARM_GRIPPER; joint++)
    {
        const ArmJointCalib_t *calib = ArmCalib_Get(joint);
        assert(calib != NULL);

        ArmRemoteCommand_t cmd = {0};

        ArmPose_t current = PoseAt(1.0f);
        ArmControl_t control = ControlAt(&current);
        cmd.jog[joint] = 1.0f;
        assert(ArmControl_Update(&control, &current, &cmd, DT_S));
        assert(control.target.joint[joint] == calib->max_val);

        current = PoseAt(0.0f);
        control = ControlAt(&current);
        cmd.jog[joint] = -1.0f;
        assert(ArmControl_Update(&control, &current, &cmd, DT_S));
        assert(control.target.joint[joint] == calib->min_val);
    }
}

static void TestJogUsesCurrentAndReleaseHolds(void)
{
    ArmPose_t stow;
    assert(ArmControl_GetPreset(ARM_PRESET_STOW, &stow));

    // current 的 J1 要和预设不同，才能区分基准用的是 current 还是旧目标
    ArmPose_t current = PoseAt(0.25f);
    assert(current.joint[ARM_J1] != stow.joint[ARM_J1]);
    ArmControl_t control = ControlAt(&current);
    ArmRemoteCommand_t cmd = {0};

    // 先用预设把目标拉远，再 jog J1：基准必须是 current，不是旧目标
    cmd.preset = ARM_PRESET_STOW;
    assert(ArmControl_Update(&control, &current, &cmd, DT_S));

    cmd.preset = ARM_PRESET_NONE;
    cmd.jog[ARM_J1] = 1.0f;
    assert(ArmControl_Update(&control, &current, &cmd, DT_S));
    AssertNear(control.target.joint[ARM_J1],
               current.joint[ARM_J1] + MaxStep(ARM_J1));

    // 松手后 target 不变，即使 current 继续变化
    ArmControl_t saved = control;
    ArmRemoteCommand_t zero = {0};
    ArmPose_t moved = PoseAt(0.75f);
    assert(ArmControl_Update(&control, &moved, &zero, DT_S));
    AssertPoseEqual(&control.target, &saved.target);
}

static void TestPresetBehaviour(void)
{
    ArmPose_t stow;
    assert(ArmControl_GetPreset(ARM_PRESET_STOW, &stow));

    ArmPose_t current = PoseAt(0.5f);
    ArmControl_t control = ControlAt(&current);
    ArmRemoteCommand_t cmd = {0};

    // 预设和 jog 同时出现：jog 被忽略，J1～J5 等于预设
    cmd.preset = ARM_PRESET_STOW;
    cmd.jog[ARM_J1] = 1.0f;
    cmd.jog[ARM_J3] = -1.0f;
    assert(ArmControl_Update(&control, &current, &cmd, DT_S));

    for (int idx = 0; idx < ARM_GRIPPER; idx++)
    {
        assert(control.target.joint[idx] == stow.joint[idx]);
    }

    // 松开预设后目标仍是预设值，让动作继续执行完
    ArmRemoteCommand_t zero = {0};
    ArmControl_t saved = control;
    assert(ArmControl_Update(&control, &current, &zero, DT_S));
    AssertPoseEqual(&control.target, &saved.target);
}

static void TestPresetKeepsGripper(void)
{
    ArmPose_t current = PoseAt(0.5f);
    ArmControl_t control = ControlAt(&current);
    ArmRemoteCommand_t cmd = {0};

    cmd.gripper_close = true;
    assert(ArmControl_Update(&control, &current, &cmd, DT_S));
    assert(control.target.joint[ARM_GRIPPER] == ARM_GRIPPER_CLOSED);

    // 切换预设时不能松开已经夹住的东西
    cmd.gripper_close = false;
    cmd.preset = ARM_PRESET_GRAB_READY;
    assert(ArmControl_Update(&control, &current, &cmd, DT_S));
    assert(control.target.joint[ARM_GRIPPER] == ARM_GRIPPER_CLOSED);

    // 预设帧里的夹爪请求照常生效
    cmd.gripper_open = true;
    assert(ArmControl_Update(&control, &current, &cmd, DT_S));
    assert(control.target.joint[ARM_GRIPPER] == ARM_GRIPPER_OPEN);
}

static void TestGripper(void)
{
    ArmPose_t current = PoseAt(0.5f);
    ArmControl_t control = ControlAt(&current);
    float initial = control.target.joint[ARM_GRIPPER];
    ArmRemoteCommand_t cmd = {0};

    // jog[ARM_GRIPPER] 不影响夹爪
    cmd.jog[ARM_GRIPPER] = 1.0f;
    assert(ArmControl_Update(&control, &current, &cmd, DT_S));
    assert(control.target.joint[ARM_GRIPPER] == initial);
    cmd.jog[ARM_GRIPPER] = 0.0f;

    cmd.gripper_close = true;
    assert(ArmControl_Update(&control, &current, &cmd, DT_S));
    assert(control.target.joint[ARM_GRIPPER] == ARM_GRIPPER_CLOSED);

    // 两个都按：保持夹紧
    cmd.gripper_open = true;
    assert(ArmControl_Update(&control, &current, &cmd, DT_S));
    assert(control.target.joint[ARM_GRIPPER] == ARM_GRIPPER_CLOSED);

    cmd.gripper_close = false;
    assert(ArmControl_Update(&control, &current, &cmd, DT_S));
    assert(control.target.joint[ARM_GRIPPER] == ARM_GRIPPER_OPEN);

    // 两个都不按：保持张开
    cmd.gripper_open = false;
    assert(ArmControl_Update(&control, &current, &cmd, DT_S));
    assert(control.target.joint[ARM_GRIPPER] == ARM_GRIPPER_OPEN);
}

static void TestZeroCommandHolds(void)
{
    ArmPose_t current = PoseAt(0.3f);
    ArmControl_t control = ControlAt(&current);
    ArmControl_t saved = control;
    ArmRemoteCommand_t zero = {0};

    for (int count = 0; count < 100; count++)
    {
        assert(ArmControl_Update(&control, &current, &zero, DT_S));
    }

    AssertPoseEqual(&control.target, &saved.target);
    assert(control.has_target);
}

static void TestInvalidInputs(void)
{
    ArmPose_t current = PoseAt(0.5f);
    ArmControl_t control = ControlAt(&current);
    ArmControl_t fresh;
    ArmRemoteCommand_t zero = {0};

    ArmControl_Init(&fresh);

    // 空指针
    assert(!ArmControl_Update(NULL, &current, &zero, DT_S));
    AssertRejected(&control, NULL, &zero, DT_S);
    AssertRejected(&control, &current, NULL, DT_S);

    // dt_s 非法；没有目标时失败也不能把 has_target 置为 true
    static const float bad_dt[] = {0.0f, -0.01f, NAN, INFINITY};
    for (size_t idx = 0; idx < sizeof(bad_dt) / sizeof(bad_dt[0]); idx++)
    {
        AssertRejected(&control, &current, &zero, bad_dt[idx]);
        AssertRejected(&fresh, &current, &zero, bad_dt[idx]);
    }

    // current 任一关节非有限
    static const float bad_joint[] = {NAN, INFINITY, -INFINITY};
    for (int joint = 0; joint < ARM_JOINT_COUNT; joint++)
    {
        for (size_t idx = 0; idx < sizeof(bad_joint) / sizeof(bad_joint[0]); idx++)
        {
            ArmPose_t bad = current;
            bad.joint[joint] = bad_joint[idx];
            AssertRejected(&control, &bad, &zero, DT_S);
            AssertRejected(&fresh, &bad, &zero, DT_S);
        }
    }

    // jog 越界或为 NaN，夹爪的 jog 也要检查
    static const float bad_jog[] = {1.0001f, -1.0001f, NAN, INFINITY};
    for (int joint = 0; joint < ARM_JOINT_COUNT; joint++)
    {
        for (size_t idx = 0; idx < sizeof(bad_jog) / sizeof(bad_jog[0]); idx++)
        {
            ArmRemoteCommand_t cmd = {0};
            cmd.jog[joint] = bad_jog[idx];
            AssertRejected(&control, &current, &cmd, DT_S);
        }
    }

    // 非法预设
    ArmRemoteCommand_t bad_preset = {0};
    bad_preset.preset = (ArmPreset_t)99;
    AssertRejected(&control, &current, &bad_preset, DT_S);

    // 已有目标本身越界：结果检查不通过
    const ArmJointCalib_t *j2 = ArmCalib_Get(ARM_J2);
    assert(j2 != NULL);
    ArmControl_t broken = control;
    broken.target.joint[ARM_J2] = j2->min_val - 1.0f;
    AssertRejected(&broken, &current, &zero, DT_S);
}

static void TestBasicApi(void)
{
    ArmControl_t control;
    ArmPose_t valid = PoseAt(0.5f);
    ArmPose_t out = PoseAt(0.25f);

    ArmControl_Init(NULL);
    ArmControl_Clear(NULL);
    ArmControl_Init(&control);

    // 没有目标时读不到，输出被清零
    assert(!ArmControl_GetTarget(&control, &out));
    assert(out.joint[ARM_J1] == 0.0f);
    assert(!ArmControl_GetTarget(NULL, &out));
    assert(!ArmControl_GetTarget(&control, NULL));

    // SetTarget 拒绝空指针和越界位姿
    ArmPose_t invalid = valid;
    invalid.joint[ARM_J2] = 0.0f;
    assert(!ArmControl_SetTarget(NULL, &valid));
    assert(!ArmControl_SetTarget(&control, NULL));
    assert(!ArmControl_SetTarget(&control, &invalid));
    assert(!control.has_target);

    assert(ArmControl_SetTarget(&control, &valid));
    assert(ArmControl_GetTarget(&control, &out));
    AssertPoseEqual(&out, &valid);

    // Clear 把目标清成全零，之后 Update 必须重新以 current 为目标，不能沿用全零
    ArmControl_Clear(&control);
    assert(!control.has_target);

    ArmPose_t current = PoseAt(0.75f);
    ArmRemoteCommand_t zero = {0};
    assert(ArmControl_Update(&control, &current, &zero, DT_S));
    AssertPoseEqual(&control.target, &current);
}

int main(void)
{
    TestBasicApi();
    TestPresets();
    TestFirstUpdateAdoptsCurrent();
    TestJogStep();
    TestJogClampedAtLimits();
    TestJogUsesCurrentAndReleaseHolds();
    TestPresetBehaviour();
    TestPresetKeepsGripper();
    TestGripper();
    TestZeroCommandHolds();
    TestInvalidInputs();

    puts("Arm control tests passed");
    return 0;
}