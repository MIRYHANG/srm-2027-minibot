//
// Created by YZH on 2026/9/26
//

#include "mecanum.h"


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

    // 从m/s到rpm换算mps_to_rpm = 60 / 轮子周长
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


/**
 * @brief 按归一化输入做麦轮混合
 * @param forward 前后，-1～1，正数向前
 * @param left 横移，-1～1，正数向左
 * @param turn 旋转，-1～1，正数逆时针
 * @param result 四轮归一化转速，绝对值都不超过 1
 * @return 输入有限且 result 非空返回 true；否则返回 false，result 非空时清零
 * @note 算法结构参考 WPILib：MecanumDrive::DriveCartesianIK 先混合，
 *       MecanumDriveWheelVelocities::Desaturate 再等比例缩小；正负号沿用本工程的公式
 */
bool Mecanum_MixNormalized(float forward, float left, float turn,
                           MecanumWheelRpm_t *result);
bool Mecanum_MixNormalized(float forward, float left, float turn,MecanumWheelRpm_t *result)
{
    if (result == NULL)
    {
        return false;
    }

    *result = (MecanumWheelRpm_t){0};

    if (!isfinite(forward) || !isfinite(left) || !isfinite(turn))
    {
        return false;
    }

    MecanumWheelRpm_t mix = {
        .fl = forward - left - turn,
        .fr = forward + left + turn,
        .rl = forward + left - turn,
        .rr = forward - left + turn,
    };

    float peak = fmaxf(fmaxf(fabsf(mix.fl), fabsf(mix.fr)),
                       fmaxf(fabsf(mix.rl), fabsf(mix.rr)));

    float scale = (peak > 1.0f) ? (1.0f / peak) : 1.0f;

    /*---相当于按比例放缩，大于1的部分缩放小于或等于1---*/
    result->fl = mix.fl * scale;
    result->fr = mix.fr * scale;
    result->rl = mix.rl * scale;
    result->rr = mix.rr * scale;
    return true;
}