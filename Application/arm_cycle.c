//
// Created by YZH on 2026/10/5.
//

#include "arm_cycle.h"

#include <math.h>
#include <stddef.h>

#include "arm_calib.h"
#include "arm_motion.h"

/**
 * @brief 把位姿逐关节换算成脉宽
 * @return 全部成功返回 true，失败时 pulse_us 可能已被部分修改
 */
static bool PoseToPulses(const ArmPose_t *pose, uint16_t pulse_us[])
{
    for (int idx = 0; idx < ARM_JOINT_COUNT; idx++)
    {
        if (!ArmCalib_ToPulseWithCalib(ArmCalib_Get(idx), pose->joint[idx], &pulse_us[idx]))
        {
            return false;
        }
    }

    return true;
}

bool ArmCycle_Init(ArmCycle_t *cycle)
{
    if (cycle == NULL)
    {
        return false;
    }

    ArmCycle_t next = {0};

    // 舵机没有位置反馈，上电只能假定机械臂在 STOW
    if (!ArmControl_GetPreset(ARM_PRESET_STOW, &next.current))
    {
        return false;
    }

    ArmControl_Init(&next.control);

    if (!ArmControl_SetTarget(&next.control, &next.current))
    {
        return false;
    }

    if (!PoseToPulses(&next.current, next.pulse_us))
    {
        return false;
    }

    next.initialized = true;
    *cycle = next;
    return true;
}

bool ArmCycle_Step(ArmCycle_t *cycle, const ArmCmd_t *cmd, float dt_s)
{
    if (cycle == NULL || cmd == NULL || !cycle->initialized)
    {
        return false;
    }

    if (!isfinite(dt_s) || dt_s <= 0.0f)
    {
        return false;
    }

    if (dt_s > ARM_CYCLE_MAX_DT_S)
    {
        dt_s = ARM_CYCLE_MAX_DT_S;
    }

    // 全部计算在局部副本里完成，失败时 cycle 保持不变
    ArmCycle_t next = *cycle;

    if (!cmd->enabled)
    {
        // 未使能、急停或失联：目标拉回当前位置，正在执行的预设也停下
        if (!ArmControl_SetTarget(&next.control, &next.current))
        {
            return false;
        }
    }
    else if (!ArmControl_Update(&next.control, &next.current, &cmd->motion, dt_s))
    {
        return false;
    }

    ArmPose_t target;

    if (!ArmControl_GetTarget(&next.control, &target))
    {
        return false;
    }

    if (!ArmMotion_Step(&next.current, &target, dt_s, &next.current))
    {
        return false;
    }

    if (!PoseToPulses(&next.current, next.pulse_us))
    {
        return false;
    }

    *cycle = next;
    return true;
}
