//
// Created by YZH on 2026/9/26.
//
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "mecanum.h"

static void ExpectWheels(const MecanumWheelRpm_t *actual,
                         float fl, float fr, float rl, float rr)
{
    const float tolerance = 0.02f;  // 允许少量浮点计算误差
    assert(fabsf(actual->fl - fl) < tolerance);
    assert(fabsf(actual->fr - fr) < tolerance);
    assert(fabsf(actual->rl - rl) < tolerance);
    assert(fabsf(actual->rr - rr) < tolerance);
}

int main(void)
{
    MecanumGeometry_t geometry = {
        .wheel_radius_m = 0.05f,
        .wheelbase_m = 0.30f,
        .track_width_m = 0.30f,
    };
    MecanumWheelRpm_t wheels;

    // 只向前
    Mecanum_CalculateWheelRpm(&geometry, 0.1f, 0.0f, 0.0f, &wheels);
    ExpectWheels(&wheels, 19.10f, 19.10f, 19.10f, 19.10f);

    // 只向左：当前代码规定 vy 正数为向左
    Mecanum_CalculateWheelRpm(&geometry, 0.0f, 0.1f, 0.0f, &wheels);
    ExpectWheels(&wheels, -19.10f, 19.10f, 19.10f, -19.10f);

    // 只逆时针旋转
    Mecanum_CalculateWheelRpm(&geometry, 0.0f, 0.0f, 1.0f, &wheels);
    ExpectWheels(&wheels, -57.30f, 57.30f, -57.30f, 57.30f);

    // 无效轮半径时，输出应全为零
    geometry.wheel_radius_m = 0.0f;
    Mecanum_CalculateWheelRpm(&geometry, 0.1f, 0.0f, 0.0f, &wheels);
    ExpectWheels(&wheels, 0.0f, 0.0f, 0.0f, 0.0f);

    puts("Mecanum tests passed");
    return 0;
}