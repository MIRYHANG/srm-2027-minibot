//
// Created by YZH on 2026/9/26
//

#ifndef XIAOSAI_MECANUM_H
#define XIAOSAI_MECANUM_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

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
 * @brief 将底盘速度换算为四个车轮的目标转速
 * @param vx_mps 向前后速度，米/秒    其中vx > 0表示向前
 * @param vy_mps 向左右速度，米/秒    其中vy > 0表示向左
 * @param wz_radps 逆时针旋转角速度，弧度/秒
 * @param result 输出的四轮目标转速，单位 RPM
 * @note 暂按俯视呈 X 型的麦轮安装方式计算；实物安装方向需核对
 */
void Mecanum_CalculateWheelRpm(const MecanumGeometry_t *geometry,
                               float vx_mps, float vy_mps, float wz_radps,
                               MecanumWheelRpm_t *result);

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
bool Mecanum_MixNormalized(float forward, float left, float turn,MecanumWheelRpm_t *result);
#endif //XIAOSAI_MECANUM_H
