//
// Created by YZH on 2026/10/5.
//
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "arm_calib.h"
#include "arm_cycle.h"

#define DT_S      0.02f
#define TOLERANCE 0.0001f

static void AssertNear(float actual, float expected)
{
    assert(fabsf(actual - expected) < TOLERANCE);
}

static void AssertPoseEqual(const ArmPose_t *actual,
                            const ArmPose_t *expected)
{
    for (int idx = 0; idx < ARM_JOINT_COUNT; idx++)
    {
        assert(actual->joint[idx] == expected->joint[idx]);
    }
}

/* 脉宽必须始终和 current 对应 */
static void AssertPulsesMatchCurrent(const ArmCycle_t *cycle)
{
    for (int idx = 0; idx < ARM_JOINT_COUNT; idx++)
    {
        uint16_t expected = 0U;
        assert(ArmCalib_ToPulseWithCalib(ArmCalib_Get(idx), cycle->current.joint[idx], &expected));
        assert(cycle->pulse_us[idx] == expected);
    }
}

static ArmCmd_t EnabledCommand(void)
{
    ArmCmd_t cmd = {0};
    cmd.enabled = true;
    return cmd;
}

static ArmCycle_t InitCycle(void)
{
    ArmCycle_t cycle;
    assert(ArmCycle_Init(&cycle));
    assert(cycle.initialized);
    return cycle;
}

/* 非法调用必须返回 false，且 cycle 逐字节不变 */
static void AssertRejected(ArmCycle_t *cycle, const ArmCmd_t *cmd,
                           float dt_s)
{
    ArmCycle_t saved;
    memcpy(&saved, cycle, sizeof(saved));

    assert(!ArmCycle_Step(cycle, cmd, dt_s));
    assert(memcmp(&saved, cycle, sizeof(saved)) == 0);
}

static void TestInitIsStow(void)
{
    ArmPose_t stow;
    assert(ArmControl_GetPreset(ARM_PRESET_STOW, &stow));

    ArmCycle_t cycle = InitCycle();
    ArmPose_t target;

    AssertPoseEqual(&cycle.current, &stow);
    assert(ArmControl_GetTarget(&cycle.control, &target));
    AssertPoseEqual(&target, &stow);
    AssertPulsesMatchCurrent(&cycle);

    assert(!ArmCycle_Init(NULL));
}

static void TestInvalidInputs(void)
{
    ArmCycle_t cycle = InitCycle();
    ArmCmd_t cmd = EnabledCommand();

    assert(!ArmCycle_Step(NULL, &cmd, DT_S));
    AssertRejected(&cycle, NULL, DT_S);

    static const float bad_dt[] = {0.0f, -0.02f, NAN, INFINITY};
    for (size_t idx = 0; idx < sizeof(bad_dt) / sizeof(bad_dt[0]); idx++)
    {
        AssertRejected(&cycle, &cmd, bad_dt[idx]);
    }

    // 没有初始化：不能从全零位姿开始输出
    ArmCycle_t zero_cycle;
    memset(&zero_cycle, 0, sizeof(zero_cycle));
    AssertRejected(&zero_cycle, &cmd, DT_S);

    // 机械臂命令非法：整步失败，保持上一次的位姿和脉宽
    cmd.motion.jog[ARM_J1] = NAN;
    AssertRejected(&cycle, &cmd, DT_S);
}

static void TestJogMovesAndUpdatesPulse(void)
{
    ArmCycle_t cycle = InitCycle();
    ArmCmd_t cmd = EnabledCommand();
    const ArmJointCalib_t *j1 = ArmCalib_Get(ARM_J1);
    assert(j1 != NULL);

    float start = cycle.current.joint[ARM_J1];
    uint16_t start_pulse = cycle.pulse_us[ARM_J1];

    cmd.motion.jog[ARM_J1] = 1.0f;
    assert(ArmCycle_Step(&cycle, &cmd, DT_S));
    AssertNear(cycle.current.joint[ARM_J1],
               start + j1->max_speed_per_s * DT_S);
    assert(cycle.pulse_us[ARM_J1] != start_pulse);
    AssertPulsesMatchCurrent(&cycle);

    // 松手后当场停住
    ArmPose_t stopped = cycle.current;
    cmd.motion.jog[ARM_J1] = 0.0f;
    for (int count = 0; count < 10; count++)
    {
        assert(ArmCycle_Step(&cycle, &cmd, DT_S));
    }
    AssertPoseEqual(&cycle.current, &stopped);
}

static void TestDtClamped(void)
{
    ArmCycle_t cycle = InitCycle();
    ArmCmd_t cmd = EnabledCommand();
    const ArmJointCalib_t *j1 = ArmCalib_Get(ARM_J1);
    assert(j1 != NULL);

    float start = cycle.current.joint[ARM_J1];

    // 任务被延迟 1 s，也只按 ARM_CYCLE_MAX_DT_S 走一步
    cmd.motion.jog[ARM_J1] = 1.0f;
    assert(ArmCycle_Step(&cycle, &cmd, 1.0f));
    AssertNear(cycle.current.joint[ARM_J1],
               start + j1->max_speed_per_s * ARM_CYCLE_MAX_DT_S);
}

static void TestPresetRunsToEnd(void)
{
    ArmPose_t grab;
    assert(ArmControl_GetPreset(ARM_PRESET_GRAB_READY, &grab));

    ArmCycle_t cycle = InitCycle();
    float gripper = cycle.current.joint[ARM_GRIPPER];
    ArmCmd_t cmd = EnabledCommand();

    // 只按一帧，之后全零命令，预设也要走完
    cmd.motion.preset = ARM_PRESET_GRAB_READY;
    assert(ArmCycle_Step(&cycle, &cmd, DT_S));
    cmd.motion.preset = ARM_PRESET_NONE;

    for (int count = 0; count < 500; count++)
    {
        assert(ArmCycle_Step(&cycle, &cmd, DT_S));
        AssertPulsesMatchCurrent(&cycle);
    }

    for (int idx = 0; idx < ARM_GRIPPER; idx++)
    {
        assert(cycle.current.joint[idx] == grab.joint[idx]);
    }
    assert(cycle.current.joint[ARM_GRIPPER] == gripper);
}

/* 预设执行到一半时停机，机械臂必须当场停住，恢复后也不能继续执行预设 */
static void AssertStopFreezes(ArmCmd_t stop_cmd)
{
    ArmCycle_t cycle = InitCycle();
    ArmCmd_t cmd = EnabledCommand();

    cmd.motion.preset = ARM_PRESET_GRAB_READY;
    for (int count = 0; count < 5; count++)
    {
        assert(ArmCycle_Step(&cycle, &cmd, DT_S));
    }

    ArmCycle_t before_stop = cycle;
    assert(ArmCycle_Step(&cycle, &stop_cmd, DT_S));
    AssertPoseEqual(&cycle.current, &before_stop.current);
    assert(memcmp(cycle.pulse_us, before_stop.pulse_us,
                  sizeof(cycle.pulse_us)) == 0);

    for (int count = 0; count < 50; count++)
    {
        assert(ArmCycle_Step(&cycle, &stop_cmd, DT_S));
    }
    AssertPoseEqual(&cycle.current, &before_stop.current);

    // 恢复使能、不按任何键：预设不会自己接着走
    ArmCmd_t idle = EnabledCommand();
    for (int count = 0; count < 50; count++)
    {
        assert(ArmCycle_Step(&cycle, &idle, DT_S));
    }
    AssertPoseEqual(&cycle.current, &before_stop.current);
}

static void TestStopFreezes(void)
{
    // 未使能时即使带着运动命令，也必须当场停住
    ArmCmd_t disabled = {0};
    disabled.motion.preset = ARM_PRESET_GRAB_READY;
    disabled.motion.jog[ARM_J1] = 1.0f;
    AssertStopFreezes(disabled);
}

static void TestGripperSpeedLimited(void)
{
    ArmCycle_t cycle = InitCycle();
    ArmCmd_t cmd = EnabledCommand();
    const ArmJointCalib_t *gripper = ArmCalib_Get(ARM_GRIPPER);
    assert(gripper != NULL);

    float start = cycle.current.joint[ARM_GRIPPER];

    cmd.motion.gripper_close = true;
    assert(ArmCycle_Step(&cycle, &cmd, DT_S));
    AssertNear(cycle.current.joint[ARM_GRIPPER],
               start + gripper->max_speed_per_s * DT_S);

    for (int count = 0; count < 100; count++)
    {
        assert(ArmCycle_Step(&cycle, &cmd, DT_S));
    }
    assert(cycle.current.joint[ARM_GRIPPER] == ARM_GRIPPER_CLOSED);
    AssertPulsesMatchCurrent(&cycle);
}

static void TestCorruptedStateRejected(void)
{
    // 状态被意外改坏时，停机路径和运动路径都必须拒绝输出
    ArmCycle_t cycle = InitCycle();
    cycle.current.joint[ARM_J2] = NAN;

    ArmCmd_t disabled = {0};
    AssertRejected(&cycle, &disabled, DT_S);

    ArmCmd_t enabled = EnabledCommand();
    AssertRejected(&cycle, &enabled, DT_S);
}

int main(void)
{
    TestInitIsStow();
    TestInvalidInputs();
    TestJogMovesAndUpdatesPulse();
    TestDtClamped();
    TestPresetRunsToEnd();
    TestStopFreezes();
    TestGripperSpeedLimited();
    TestCorruptedStateRejected();

    puts("Arm cycle tests passed");
    return 0;
}