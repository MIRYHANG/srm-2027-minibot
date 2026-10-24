//
// Created by YZH on 2026/10/24.
//

#ifndef XIAOSAI_IMU_YAW_H
#define XIAOSAI_IMU_YAW_H

#include <stdbool.h>
#include <stdint.h>

#define IMU_BIAS_SAMPLES   400U  // 零偏标定的采样数，200 Hz 下约 2 s
#define IMU_STILL_SPAN_DPS 2.0f  // 标定期间角速度最大值与最小值之差超过它，说明车在动，重新标定

typedef struct
{
    float bias_dps;   // 陀螺仪零偏，标定完成后固定
    float yaw_deg;    // 积分得到的航向，-180～180，逆时针为正
    float rate_dps;   // 减去零偏后的角速度，逆时针为正
    bool calibrated;  // 零偏标定完成，yaw_deg 和 rate_dps 才有效
    float sum_dps;    // 以下为标定过程中的累计量
    float min_dps;
    float max_dps;
    uint32_t count;
} ImuYaw_t;

/**
 * @brief 清空状态，重新开始零偏标定
 */
void ImuYaw_Init(ImuYaw_t *imu);

/**
 * @brief 输入一次竖直轴角速度
 * @param raw_dps 未减零偏的角速度，单位 °/s，已按安装方向换算成逆时针为正
 * @param dt_s 距上次调用的时间，单位 s
 * @return 已完成标定返回 true；标定中或参数非法返回 false
 * @note 标定期间车必须静止；检测到转动会自动从头开始
 */
bool ImuYaw_Update(ImuYaw_t *imu, float raw_dps, float dt_s);

/**
 * @brief 把角度换算到 -180～180
 */
float ImuYaw_Wrap(float deg);

#endif //XIAOSAI_IMU_YAW_H
