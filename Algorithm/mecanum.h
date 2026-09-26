//
// Created by YZH on 2026/9/26.
//

#ifndef XIAOSAI_MECANUM_H
#define XIAOSAI_MECANUM_H

typedef struct
{
    float wheel_radius_m;  // 轮半径，米
    float wheelbase_m;     // 前后轮中心距离，米
    float track_width_m;   // 左右轮中心距离，米
} MecanumGeometry_t;

typedef struct
{
    float fl;
    float fr;
    float rl;
    float rr;
} MecanumWheelRpm_t;

/**
 * @brief 将底盘速度换算为四个车轮的目标转速。
 * @param vx_mps 向前后速度，米/秒。    其中vx > 0表示向前
 * @param vy_mps 向左右速度，米/秒。    其中vy > 0表示向左
 * @param wz_radps 逆时针旋转角速度，弧度/秒。
 * @param result 输出的四轮目标转速，单位 RPM。
 * @note 暂按俯视呈 X 型的麦轮安装方式计算；实物安装方向需核对。
 */
void Mecanum_CalculateWheelRpm(const MecanumGeometry_t *geometry,
                               float vx_mps, float vy_mps, float wz_radps,
                               MecanumWheelRpm_t *result);

#endif //XIAOSAI_MECANUM_H
