//
// Created by YZH on 2026/9/30.
//

#include "arm_control.h"

#include <math.h>
#include <stddef.h>

#include "arm_calib.h"

/*-----------------手臂伸出去、夹爪对准地面矿石的姿态，到了矿石旁边先切换到这个姿态，再用摇杆微调--------------*/
static const ArmPose_t PRESET_GRAB_READY = {
    {90.0f, 60.0f, 120.0f, 135.0f, 90.0f, ARM_GRIPPER_OPEN} // TODO：实物测量
};

/*-----------------把机械臂收拢起来的姿态，用于行驶和运输时，重心低--------------------*/
static const ArmPose_t PRESET_STOW = {
    {90.0f, 30.0f, 30.0f, 90.0f, 90.0f, ARM_GRIPPER_OPEN}   // TODO：实物测量
};

void ArmControl_Init(ArmControl_t *control)
{
    if (control == NULL)
    {
        return;
    }

    *control = (ArmControl_t){0};
}

bool ArmControl_SetTarget(ArmControl_t *control, const ArmPose_t *pose)
{
    if (control == NULL || pose == NULL)
    {
        return false;
    }

    if (!ArmPose_IsValid(pose))
    {
        return false;
    }

    control->target = *pose;
    control->has_target = true;
    return true;
}

bool ArmControl_GetTarget(const ArmControl_t *control, ArmPose_t *out_pose)
{
    if (out_pose == NULL)
    {
        return false;
    }

    *out_pose = (ArmPose_t){0};

    if (control == NULL)
    {
        return false;
    }

    if (!control->has_target)
    {
        return false;
    }

    // 将六个目标值复制给调用者
    *out_pose = control->target;
    return true;
}

void ArmControl_Clear(ArmControl_t *control)
{
    if (control == NULL)
    {
        return;
    }

    // 删除旧目标，避免之后误用
    control->target = (ArmPose_t){0};
    control->has_target = false;
}

bool ArmControl_GetPreset(ArmPreset_t preset, ArmPose_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    switch (preset)
    {
        case ARM_PRESET_GRAB_READY:
            *out = PRESET_GRAB_READY;
            return true;
        case ARM_PRESET_STOW:
            *out = PRESET_STOW;
            return true;
        default:
            return false;
    }
}

/**
 * @brief 检查 Update 的全部输入
 * @return 指针非空、dt_s 有限且为正、current 各关节有限、jog 均在 -1～1 内时返回 true
 */
static bool UpdateInputsValid(const ArmControl_t *control,
                              const ArmPose_t *current,
                              const ArmMotionCmd_t *cmd,
                              float dt_s)
{
    if (control == NULL || current == NULL || cmd == NULL)
    {
        return false;
    }

    if (!isfinite(dt_s) || dt_s <= 0.0f)
    {
        return false;
    }

    for (int idx = 0; idx < ARM_JOINT_COUNT; idx++)
    {
        if (!isfinite(current->joint[idx]))
        {
            return false;
        }

        // NaN 的比较结果为 false，也会被拒绝
        if (!(cmd->jog[idx] >= -1.0f && cmd->jog[idx] <= 1.0f))
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief 将位姿逐关节裁剪到标定限位
 * @return 全部成功返回 true，失败时 out 可能已被部分修改
 */
static bool ClampPose(const ArmPose_t *in, ArmPose_t *out)
{
    for (int idx = 0; idx < ARM_JOINT_COUNT; idx++)
    {
        if (!ArmCalib_Clamp(idx, in->joint[idx], &out->joint[idx]))
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief 把预设位姿的 J1～J5 写入目标，夹爪保持不变
 * @note 夹爪不跟随预设，避免切换姿态时松开已经夹住的矿
 */
static bool ApplyPreset(ArmPreset_t preset, ArmPose_t *target)
{
    ArmPose_t pose;

    if (!ArmControl_GetPreset(preset, &pose))
    {
        return false;
    }

    for (int idx = 0; idx < ARM_GRIPPER; idx++)
    {
        target->joint[idx] = pose.joint[idx];
    }

    return true;
}

/**
 * @brief 对 jog 不为零的 J1～J5，以当前位姿为基准计算新目标
 * @note jog 为零的关节保持原目标，所以松手后预设动作会继续执行完
 */
static bool ApplyJog(const ArmPose_t *current, const float jog[],
                     float dt_s, ArmPose_t *target)
{
    for (int idx = 0; idx < ARM_GRIPPER; idx++)
    {
        if (jog[idx] == 0.0f)
        {
            continue;
        }

        const ArmJointCalib_t *calib = ArmCalib_Get(idx);
        if (calib == NULL)
        {
            return false;
        }

        // 基准用 current 而不是旧目标，松手后目标就停在当前位置
        float raw = current->joint[idx] +
                    jog[idx] * calib->max_speed_per_s * dt_s;

        if (!ArmCalib_Clamp(idx, raw, &target->joint[idx]))
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief 按夹紧 / 松开请求更新夹爪目标，jog[ARM_GRIPPER] 一律忽略
 * @note 两个都按或都不按时保持不变
 */
static void ApplyGripper(const ArmMotionCmd_t *cmd, ArmPose_t *target)
{
    if (cmd->gripper_close == cmd->gripper_open)
    {
        return;
    }

    target->joint[ARM_GRIPPER] = cmd->gripper_close ? ARM_GRIPPER_CLOSED
                                                    : ARM_GRIPPER_OPEN;
}

bool ArmControl_Update(ArmControl_t *control, const ArmPose_t *current,
                       const ArmMotionCmd_t *cmd, float dt_s)
{
    if (!UpdateInputsValid(control, current, cmd, dt_s))
    {
        return false;
    }

    // 全部计算在局部副本里完成，失败时 control 保持不变
    ArmControl_t next = *control;

    if (!next.has_target)
    {
        if (!ClampPose(current, &next.target))
        {
            return false;
        }
        next.has_target = true;
    }

    // 有预设请求时，本帧忽略 jog
    bool ok = (cmd->preset != ARM_PRESET_NONE)
                  ? ApplyPreset(cmd->preset, &next.target)
                  : ApplyJog(current, cmd->jog, dt_s, &next.target);
    if (!ok)
    {
        return false;
    }

    ApplyGripper(cmd, &next.target);

    if (!ArmPose_IsValid(&next.target))
    {
        return false;
    }

    *control = next;
    return true;
}