//
// Created by YZH on 2026/9/26
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

/* 归一化混合：只看比例，不需要底盘尺寸 */
static void TestMixNormalized(void)
{
    MecanumWheelRpm_t wheels;

    // 单一方向推满：和 Mecanum_CalculateWheelRpm 的正负号一致
    assert(Mecanum_MixNormalized(1.0f, 0.0f, 0.0f, &wheels));
    ExpectWheels(&wheels, 1.0f, 1.0f, 1.0f, 1.0f);
    assert(Mecanum_MixNormalized(0.0f, 1.0f, 0.0f, &wheels));
    ExpectWheels(&wheels, -1.0f, 1.0f, 1.0f, -1.0f);
    assert(Mecanum_MixNormalized(0.0f, 0.0f, 1.0f, &wheels));
    ExpectWheels(&wheels, -1.0f, 1.0f, -1.0f, 1.0f);

    // 没超过 1：原样输出
    assert(Mecanum_MixNormalized(0.5f, 0.0f, 0.0f, &wheels));
    ExpectWheels(&wheels, 0.5f, 0.5f, 0.5f, 0.5f);

    // 前进加左移推满：原始值 (0, 2, 2, 0)，同比例缩到 (0, 1, 1, 0)
    assert(Mecanum_MixNormalized(1.0f, 1.0f, 0.0f, &wheels));
    ExpectWheels(&wheels, 0.0f, 1.0f, 1.0f, 0.0f);

    // 三个方向都推满：原始值 (-1, 3, 1, 1)，除以 3
    assert(Mecanum_MixNormalized(1.0f, 1.0f, 1.0f, &wheels));
    ExpectWheels(&wheels, -1.0f / 3.0f, 1.0f, 1.0f / 3.0f, 1.0f / 3.0f);

    // 非有限输入：返回 false，输出清零
    assert(!Mecanum_MixNormalized(NAN, 0.0f, 0.0f, &wheels));
    ExpectWheels(&wheels, 0.0f, 0.0f, 0.0f, 0.0f);
    assert(!Mecanum_MixNormalized(0.0f, INFINITY, 0.0f, &wheels));
    assert(!Mecanum_MixNormalized(0.0f, 0.0f, -INFINITY, &wheels));
    assert(!Mecanum_MixNormalized(1.0f, 0.0f, 0.0f, NULL));
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

    TestMixNormalized();
    puts("Mecanum tests passed");
    return 0;
}
