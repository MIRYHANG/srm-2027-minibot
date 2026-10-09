//
// Created by YZH on 2026/10/8.
//
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "robot_cmd.h"
#include "safety_input.h"

/* 操作手打开使能、推着摇杆、按着 J1 的一帧 */
static RemoteCommand_t EnableFrame(void)
{
    RemoteCommand_t cmd = {0};
    cmd.enabled = true;
    cmd.forward = 0.5f;
    cmd.left = -0.25f;
    cmd.turn = 1.0f;
    cmd.arm.jog[ARM_J1] = 1.0f;
    return cmd;
}

static RemoteCommand_t DisableFrame(void)
{
    RemoteCommand_t cmd = EnableFrame();
    cmd.enabled = false;
    return cmd;
}

static RobotCmdOutput_t Step(RobotCmd_t *robot, bool online,
                             const RemoteCommand_t *cmd, uint8_t safety)
{
    RobotCmdOutput_t out;
    memset(&out, 0xA5, sizeof(out));
    RobotCmd_Update(robot, online, cmd, safety, &out);
    return out;
}

static void AssertStopped(const RobotCmdOutput_t *out)
{
    assert(!out->chassis.enabled);
    assert(out->chassis.forward == 0.0f);
    assert(out->chassis.left == 0.0f);
    assert(out->chassis.turn == 0.0f);
    assert(!out->arm.enabled);
    for (int idx = 0; idx < ARM_JOINT_COUNT; idx++)
    {
        assert(out->arm.motion.jog[idx] == 0.0f);
    }
}

static void AssertRunning(const RobotCmdOutput_t *out, const RemoteCommand_t *cmd)
{
    assert(out->chassis.enabled);
    assert(out->chassis.forward == cmd->forward);
    assert(out->chassis.left == cmd->left);
    assert(out->chassis.turn == cmd->turn);
    assert(out->arm.enabled);
    assert(out->arm.motion.jog[ARM_J1] == cmd->arm.jog[ARM_J1]);
}

/* 先失能一帧、再使能一帧，得到正在运行的状态 */
static RobotCmd_t RunningRobot(void)
{
    RobotCmd_t robot;
    RemoteCommand_t off = DisableFrame();
    RemoteCommand_t on = EnableFrame();

    RobotCmd_Init(&robot);
    (void)Step(&robot, true, &off, 0U);
    RobotCmdOutput_t out = Step(&robot, true, &on, 0U);
    AssertRunning(&out, &on);
    return robot;
}

static void TestBootNeedsDisableFirst(void)
{
    RobotCmd_t robot;
    RobotCmd_Init(&robot);
    assert(!robot.enabled);
    assert(robot.gear == ROBOT_GEAR_MIN);

    // 上电时手柄已经锁存在使能：一直不动
    RemoteCommand_t on = EnableFrame();
    for (int count = 0; count < 10; count++)
    {
        RobotCmdOutput_t out = Step(&robot, true, &on, 0U);
        AssertStopped(&out);
    }

    // 先失能再使能：开始运动
    RemoteCommand_t off = DisableFrame();
    RobotCmdOutput_t out = Step(&robot, true, &off, 0U);
    AssertStopped(&out);
    out = Step(&robot, true, &on, 0U);
    AssertRunning(&out, &on);
}

static void TestOperatorDisable(void)
{
    // 按失能键：立刻停止；再按使能键即可恢复
    RobotCmd_t robot = RunningRobot();
    RemoteCommand_t off = DisableFrame();
    RemoteCommand_t on = EnableFrame();

    RobotCmdOutput_t out = Step(&robot, true, &off, 0U);
    AssertStopped(&out);
    assert(!robot.enabled);

    out = Step(&robot, true, &on, 0U);
    AssertRunning(&out, &on);

    // 停机请求也算失能
    RemoteCommand_t stop = EnableFrame();
    stop.stop_requested = true;
    out = Step(&robot, true, &stop, 0U);
    AssertStopped(&out);
    assert(!robot.enabled);
    out = Step(&robot, true, &on, 0U);
    AssertRunning(&out, &on);
}

static void TestSafetyLatches(void)
{
    static const uint8_t causes[] = {SAFETY_ESTOP, SAFETY_FAULT_FL, SAFETY_FAULT_RR};

    for (size_t idx = 0U; idx < sizeof(causes) / sizeof(causes[0]); idx++)
    {
        RobotCmd_t robot = RunningRobot();
        RemoteCommand_t off = DisableFrame();
        RemoteCommand_t on = EnableFrame();

        // 急停或驱动故障：立刻停止
        RobotCmdOutput_t out = Step(&robot, true, &on, causes[idx]);
        AssertStopped(&out);

        // 有效期间操作手的失能、使能都不算数
        (void)Step(&robot, true, &off, causes[idx]);
        out = Step(&robot, true, &on, causes[idx]);
        AssertStopped(&out);

        // 解除后手柄仍锁存在使能：不会自己恢复
        for (int count = 0; count < 10; count++)
        {
            out = Step(&robot, true, &on, 0U);
            AssertStopped(&out);
        }

        // 先失能、再使能才恢复
        out = Step(&robot, true, &off, 0U);
        AssertStopped(&out);
        out = Step(&robot, true, &on, 0U);
        AssertRunning(&out, &on);
    }
}

static void TestOfflineKeepsEnabled(void)
{
    RobotCmd_t robot = RunningRobot();
    RemoteCommand_t on = EnableFrame();

    // 掉线：停止输出，但使能状态不变
    for (int count = 0; count < 10; count++)
    {
        RobotCmdOutput_t out = Step(&robot, false, &on, 0U);
        AssertStopped(&out);
        assert(robot.enabled);
    }

    // remote 为 NULL 也按掉线处理
    RobotCmdOutput_t out = Step(&robot, true, NULL, 0U);
    AssertStopped(&out);
    assert(robot.enabled);

    // 恢复在线：不用重新使能，直接继续
    out = Step(&robot, true, &on, 0U);
    AssertRunning(&out, &on);

    // 失能状态下掉线：仍然是失能，恢复后还要先失能再使能
    RobotCmd_t locked;
    RobotCmd_Init(&locked);
    (void)Step(&locked, false, &on, 0U);
    out = Step(&locked, true, &on, 0U);
    AssertStopped(&out);
}

static void TestArmStop(void)
{
    RobotCmd_t robot = RunningRobot();
    RemoteCommand_t cmd = EnableFrame();
    cmd.arm_stop = true;

    // 机械臂停住，底盘照常
    RobotCmdOutput_t out = Step(&robot, true, &cmd, 0U);
    assert(out.chassis.enabled);
    assert(out.chassis.forward == cmd.forward);
    assert(!out.arm.enabled);
    assert(out.arm.motion.jog[ARM_J1] == 0.0f);
    assert(robot.enabled);

    // 解除机械臂停止
    cmd.arm_stop = false;
    out = Step(&robot, true, &cmd, 0U);
    AssertRunning(&out, &cmd);
}

static void TestGear(void)
{
    RobotCmd_t robot = RunningRobot();
    RemoteCommand_t cmd = EnableFrame();
    RobotCmdOutput_t out;

    // 每按下一次加 1 挡，按住不重复换挡，3 挡后回到 1 挡
    static const uint8_t expected[] = {2U, 3U, 1U, 2U};
    for (size_t idx = 0U; idx < sizeof(expected) / sizeof(expected[0]); idx++)
    {
        cmd.gear_button = true;
        for (int count = 0; count < 5; count++)
        {
            out = Step(&robot, true, &cmd, 0U);
            assert(out.chassis.gear == expected[idx]);
        }

        cmd.gear_button = false;
        out = Step(&robot, true, &cmd, 0U);
        assert(out.chassis.gear == expected[idx]);
    }

    // 失能后挡位保留用于显示，失能期间按换挡键不起作用
    RemoteCommand_t off = DisableFrame();
    out = Step(&robot, true, &off, 0U);
    assert(out.chassis.gear == 2U);
    off.gear_button = true;
    out = Step(&robot, true, &off, 0U);
    assert(out.chassis.gear == 2U);

    // 重新使能回到 1 挡；使能时换挡键正按着，不算一次换挡
    cmd.gear_button = true;
    out = Step(&robot, true, &cmd, 0U);
    assert(out.chassis.enabled);
    assert(out.chassis.gear == ROBOT_GEAR_MIN);
    out = Step(&robot, true, &cmd, 0U);
    assert(out.chassis.gear == ROBOT_GEAR_MIN);
}

static void TestNullArguments(void)
{
    RemoteCommand_t on = EnableFrame();

    RobotCmd_Init(NULL);

    // robot 为 NULL：输出停止
    RobotCmdOutput_t out;
    memset(&out, 0xA5, sizeof(out));
    RobotCmd_Update(NULL, true, &on, 0U, &out);
    AssertStopped(&out);

    // out 为 NULL：不修改 robot
    RobotCmd_t robot = RunningRobot();
    RobotCmd_t saved = robot;
    RobotCmd_Update(&robot, true, &on, SAFETY_ESTOP, NULL);
    assert(memcmp(&robot, &saved, sizeof(robot)) == 0);
}

int main(void)
{
    TestBootNeedsDisableFirst();
    TestOperatorDisable();
    TestSafetyLatches();
    TestOfflineKeepsEnabled();
    TestArmStop();
    TestGear();
    TestNullArguments();

    puts("Robot cmd tests passed");
    return 0;
}