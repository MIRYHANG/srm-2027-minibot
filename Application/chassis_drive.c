//
// Created by YZH on 2026/10/10.
//

#include "chassis_drive.h"

/**
 * @brief 依旧检测边界的函数
 */
static bool AxisValid(float value)
{
    return value >= -1.0f && value <= 1.0f;
}

float ChassisDrive_GearMaxRpm(uint8_t gear)
{
    switch (gear)
    {
        case 1:
            return CHASSIS_GEAR1_MAX_RPM;
        case 2:
            return CHASSIS_GEAR2_MAX_RPM;
        case 3:
            return CHASSIS_GEAR3_MAX_RPM;
        default:
            return 0.0f;
    }
}

bool ChassisDrive_ToWheelRpm(const ChassisCmd_t *cmd, MecanumWheelRpm_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    *out = (MecanumWheelRpm_t){0};

    if (cmd == NULL)
    {
        return false;
    }

    if (!cmd->enabled)
    {
        return true;
    }

    float max_rpm = ChassisDrive_GearMaxRpm(cmd->gear);

    if (max_rpm <= 0.0f || !AxisValid(cmd->forward) || !AxisValid(cmd->left) || !AxisValid(cmd->turn))
    {
        return false;
    }

    MecanumWheelRpm_t mix;

    if (!Mecanum_MixNormalized(cmd->forward,cmd->left,cmd->turn * CHASSIS_TURN_SCALE,&mix))
    {
        return false;
    }

    out->fl = mix.fl * max_rpm;
    out->fr = mix.fr * max_rpm;
    out->rl = mix.rl * max_rpm;
    out->rr = mix.rr * max_rpm;
    return true;
}

void ChassisRamp_Reset(ChassisRamp_t *ramp)
{
    if (ramp == NULL)
    {
        return;
    }

    *ramp = (ChassisRamp_t){0};
}

/**
 * @brief 四个轮速都是有限的普通数字
 */
static bool WheelsFinite(const MecanumWheelRpm_t *w)
{
    return isfinite(w->fl) && isfinite(w->fr) && isfinite(w->rl) && isfinite(w->rr);
}

bool ChassisRamp_Step(ChassisRamp_t *ramp, const MecanumWheelRpm_t *target,
                      float dt_s, MecanumWheelRpm_t *out)
{
    if (ramp == NULL || out == NULL)
    {
        return false;
    }

    if (target == NULL || !WheelsFinite(target) || !isfinite(dt_s) || dt_s <= 0.0f)
    {
        *out = ramp->current;
        return false;
    }

    MecanumWheelRpm_t delta = {
        .fl = target->fl - ramp->current.fl,
        .fr = target->fr - ramp->current.fr,
        .rl = target->rl - ramp->current.rl,
        .rr = target->rr - ramp->current.rr,
    };

    float max_step = CHASSIS_WHEEL_RPM_PER_S * dt_s;
    float peak = fmaxf(fmaxf(fabsf(delta.fl), fabsf(delta.fr)),
                       fmaxf(fabsf(delta.rl), fabsf(delta.rr)));

    float scale = (peak > max_step) ? (max_step / peak) : 1.0f;

    ramp->current.fl += delta.fl * scale;
    ramp->current.fr += delta.fr * scale;
    ramp->current.rl += delta.rl * scale;
    ramp->current.rr += delta.rr * scale;

    *out = ramp->current;
    return true;
}