//
// Created by YZH on 2026/10/24.
//

#include "heading_hold.h"
#include <math.h>
#include <stddef.h>

#include "chassis_drive.h"
#include "imu_yaw.h"

void HeadingHold_Reset(HeadingHold_t *hold)
{
    if (hold == NULL)
    {
        return;
    }

    *hold = (HeadingHold_t){0};
}

static float Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

float HeadingHold_Update(HeadingHold_t *hold, const ChassisCmd_t *cmd, bool imu_valid,
                         float yaw_deg, float rate_dps, float dt_s)
{
    if (hold == NULL || cmd == NULL)
    {
        return 0.0f;
    }

    if (!cmd->enabled)
    {
        HeadingHold_Reset(hold);
        return 0.0f;
    }

    // 操作手在自转：照常转，松开后先等车身停下
    if (cmd->turn != 0.0f)
    {
        hold->locked = false;
        hold->settle_s = HEADING_SETTLE_S;
        return cmd->turn;
    }

    bool moving = fabsf(cmd->forward) >= HEADING_MOVE_DEADBAND ||
                  fabsf(cmd->left) >= HEADING_MOVE_DEADBAND;

    if (!moving || !imu_valid || !isfinite(yaw_deg) || !isfinite(rate_dps) ||
        !isfinite(dt_s) || dt_s <= 0.0f)
    {
        hold->locked = false;
        return 0.0f;
    }

    if (!hold->locked)
    {
        hold->settle_s -= dt_s;
        if (hold->settle_s > 0.0f)
        {
            return 0.0f;
        }
        hold->locked = true;
        hold->settle_s = 0.0f;
        hold->target_deg = yaw_deg;
    }

    // 车往顺时针偏时 error 为正，输出逆时针自转把它拉回来
    float error_deg = ImuYaw_Wrap(hold->target_deg - yaw_deg);
    float correction_rpm = HEADING_KP_RPM_PER_DEG * error_deg -
                           HEADING_KD_RPM_PER_DPS * rate_dps;
    correction_rpm = Clamp(correction_rpm, HEADING_MAX_RPM);

    // turn 会再乘挡位转速和自转系数，这里除回去，修正力度不随挡位变化
    float full_rpm = ChassisDrive_GearMaxRpm(cmd->gear) * CHASSIS_TURN_SCALE;
    if (full_rpm <= 0.0f)
    {
        return 0.0f;
    }

    return Clamp(correction_rpm / full_rpm, 1.0f);
}
