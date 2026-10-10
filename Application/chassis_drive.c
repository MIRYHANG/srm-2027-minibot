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

    if (max_rpm < 0.0f || !AxisValid(cmd->forward) || !AxisValid(cmd->left) || !AxisValid(cmd->turn))
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