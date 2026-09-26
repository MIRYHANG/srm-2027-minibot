//
// Created by YZH on 2026/9/26.
//

#include "mecanum.h"
#include <stddef.h>

void Mecanum_CalculateWheelRpm(const MecanumGeometry_t *geometry,
                               float vx_mps, float vy_mps, float wz_radps,
                               MecanumWheelRpm_t *result) {
    if (result == NULL)
    {
        return;
    }

    *result = (MecanumWheelRpm_t){0};

    if (geometry == NULL ||
        geometry->wheel_radius_m <= 0.0f ||
        geometry->wheelbase_m <= 0.0f ||
        geometry->track_width_m <= 0.0f)
    {
        return;
    }

    // rotation = (半轴距 + 半轮距) × 旋转角速度
    float rotation = 0.5f * (geometry->wheelbase_m +
                             geometry->track_width_m) * wz_radps;

    // 从m/s 到RPM换算 mps_to_rpm = 60 / 轮子周长
    float mps_to_rpm = 60.0f /
        (2.0f * 3.14159265f * geometry->wheel_radius_m);

    // FL = vx - vy - k*wz;
    // FR = vx + vy + k*wz;
    // RL = vx + vy - k*wz;
    // RR = vx - vy + k*wz;
    result->fl = (vx_mps - vy_mps - rotation) * mps_to_rpm;
    result->fr = (vx_mps + vy_mps + rotation) * mps_to_rpm;
    result->rl = (vx_mps + vy_mps - rotation) * mps_to_rpm;
    result->rr = (vx_mps - vy_mps + rotation) * mps_to_rpm;
}
