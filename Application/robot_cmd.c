//
// Created by YZH on 2026/10/8.
//

#include "robot_cmd.h"
#include <stddef.h>

void RobotCmd_Init(RobotCmd_t *robot)
{
    if (robot == NULL)
    {
        return;
    }

    *robot = (RobotCmd_t){.gear = ROBOT_GEAR_MIN};
}

/**
 * @brief 失能状态下处理操作手的命令：先收到一帧未使能，再收到使能才解锁
 */
static void HandleDisabled(RobotCmd_t *robot, const RemoteCommand_t *remote)
{
    /*---------------使能开关打开且未处于停机状态-----------------*/
    bool wants_enable = remote->enabled && !remote->stop_requested;

    if (!wants_enable)
    {
        robot->seen_disable = true;
        return;
    }

    if (robot->seen_disable)
    {
        robot->enabled = true;
        robot->seen_disable = false;
        robot->gear = ROBOT_GEAR_MIN;

        // 使能时换挡键正按着，不算一次换挡
        robot->last_gear = remote->gear_button;
    }
}

/**
 * @brief 使能状态下处理操作手的命令，收到未使能或停机请求就失能
 */
static void HandleEnabled(RobotCmd_t *robot, const RemoteCommand_t *remote)
{
    if (!remote->enabled || remote->stop_requested)
    {
        robot->enabled = false;
        robot->seen_disable = true;
    }
}

/**
 * @brief 换挡键按下的瞬间挡位加 1，超过最高挡回到 1 挡
 */
static void UpdateGear(RobotCmd_t *robot, bool gear_button)
{
    if (gear_button && !robot->last_gear)
    {
        robot->gear = (robot->gear >= ROBOT_GEAR_MAX) ? (uint8_t)ROBOT_GEAR_MIN : (uint8_t)(robot->gear + 1U);
    }

    robot->last_gear = gear_button;
}

/**
 * @brief 使能且在线时，把遥控命令转成底盘和机械臂命令
 */
static void BuildOutput(const RobotCmd_t *robot, const RemoteCommand_t *remote,
                        RobotCmdOutput_t *out)
{
    out->chassis.enabled = true;
    out->chassis.forward = remote->forward;
    out->chassis.left = remote->left;
    out->chassis.turn = remote->turn;
    out->chassis.gear = robot->gear;

    /* ------机械臂单独停止时只停机械臂-------- */
    if (!remote->arm_stop)
    {
        out->arm.enabled = true;
        out->arm.motion = remote->arm;
    }
}

void RobotCmd_Update(RobotCmd_t *robot, bool online, const RemoteCommand_t *remote,
                     uint8_t safety_active, RobotCmdOutput_t *out)
{
    if (out == NULL)
    {
        return;
    }
    *out = (RobotCmdOutput_t){0};

    if (robot == NULL)
    {
        return;
    }

    out->chassis.gear = robot->gear;
    bool has_remote = online && remote != NULL;

    if (safety_active != 0)
    {
        /* --急停和驱动故障直接失能；解除后必须先收到一帧未使能，手柄锁存在使能时不会自己恢复-- */
        robot->enabled = false;
        robot->seen_disable = false;
        return;
    }

    if (!has_remote)
    {
        return;
    }

    if (robot->enabled)
    {
        HandleEnabled(robot, remote);
    }
    else
    {
        HandleDisabled(robot, remote);
    }

    if (!robot->enabled)
    {
        return;
    }

    UpdateGear(robot, remote->gear_button);
    BuildOutput(robot, remote, out);
}