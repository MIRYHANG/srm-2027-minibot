//
// Created by YZH on 2026/10/24.
//

#include "imu_yaw.h"
#include <math.h>
#include <stddef.h>

void ImuYaw_Init(ImuYaw_t *imu)
{
    if (imu == NULL)
    {
        return;
    }

    *imu = (ImuYaw_t){0};
}

float ImuYaw_Wrap(float deg)
{
    if (!isfinite(deg))
    {
        return 0.0f;
    }

    float wrapped = fmodf(deg + 180.0f, 360.0f);
    if (wrapped < 0.0f)
    {
        wrapped += 360.0f;
    }
    return wrapped - 180.0f;
}

/**
 * @brief 累计一个标定样本；车在动时清空重来
 */
static void CalibrateStep(ImuYaw_t *imu, float raw_dps)
{
    if (imu->count == 0U)
    {
        imu->min_dps = raw_dps;
        imu->max_dps = raw_dps;
    }
    else
    {
        imu->min_dps = fminf(imu->min_dps, raw_dps);
        imu->max_dps = fmaxf(imu->max_dps, raw_dps);
    }

    if (imu->max_dps - imu->min_dps > IMU_STILL_SPAN_DPS)
    {
        ImuYaw_Init(imu);
        return;
    }

    imu->sum_dps += raw_dps;
    imu->count++;

    if (imu->count >= IMU_BIAS_SAMPLES)
    {
        imu->bias_dps = imu->sum_dps / (float)imu->count;
        imu->yaw_deg = 0.0f;
        imu->rate_dps = 0.0f;
        imu->calibrated = true;
    }
}

bool ImuYaw_Update(ImuYaw_t *imu, float raw_dps, float dt_s)
{
    if (imu == NULL || !isfinite(raw_dps) || !isfinite(dt_s) || dt_s <= 0.0f)
    {
        return false;
    }

    if (!imu->calibrated)
    {
        CalibrateStep(imu, raw_dps);
        return false;
    }

    imu->rate_dps = raw_dps - imu->bias_dps;
    imu->yaw_deg = ImuYaw_Wrap(imu->yaw_deg + imu->rate_dps * dt_s);
    return true;
}
