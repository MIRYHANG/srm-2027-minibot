//
// Created by YZH on 2026/10/24.
//
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "imu_yaw.h"

#define DT_S 0.005f
#define TOLERANCE 0.01f

static void AssertNear(float actual, float expected, float tol)
{
    assert(fabsf(actual - expected) < tol);
}

/* 送入 count 个固定读数，返回最后一次的返回值 */
static bool Feed(ImuYaw_t *imu, float raw_dps, uint32_t count)
{
    bool ok = false;
    for (uint32_t idx = 0U; idx < count; idx++)
    {
        ok = ImuYaw_Update(imu, raw_dps, DT_S);
    }
    return ok;
}

static void TestBiasCalibration(void)
{
    ImuYaw_t imu;
    ImuYaw_Init(&imu);

    // 标定期间返回 false，满 IMU_BIAS_SAMPLES 个样本后完成
    assert(!Feed(&imu, 0.5f, IMU_BIAS_SAMPLES - 1U));
    assert(!imu.calibrated);
    assert(!ImuYaw_Update(&imu, 0.5f, DT_S));
    assert(imu.calibrated);
    AssertNear(imu.bias_dps, 0.5f, TOLERANCE);
    AssertNear(imu.yaw_deg, 0.0f, TOLERANCE);

    // 之后减去零偏：静止时航向不漂
    assert(Feed(&imu, 0.5f, 1000U));
    AssertNear(imu.yaw_deg, 0.0f, TOLERANCE);
    AssertNear(imu.rate_dps, 0.0f, TOLERANCE);
}

static void TestMotionRestartsCalibration(void)
{
    ImuYaw_t imu;
    ImuYaw_Init(&imu);

    (void)Feed(&imu, 0.0f, 100U);
    assert(imu.count == 100U);

    // 标定途中车被转动：清空重来
    (void)ImuYaw_Update(&imu, IMU_STILL_SPAN_DPS + 1.0f, DT_S);
    assert(!imu.calibrated);
    assert(imu.count == 0U);

    (void)Feed(&imu, 0.2f, IMU_BIAS_SAMPLES);
    assert(imu.calibrated);
    AssertNear(imu.bias_dps, 0.2f, TOLERANCE);
}

static void TestIntegrationAndWrap(void)
{
    ImuYaw_t imu;
    ImuYaw_Init(&imu);
    (void)Feed(&imu, 1.0f, IMU_BIAS_SAMPLES);

    // 减去零偏后 90 °/s 转 1 s：航向 90°
    assert(Feed(&imu, 91.0f, 200U));
    AssertNear(imu.yaw_deg, 90.0f, 0.05f);
    AssertNear(imu.rate_dps, 90.0f, TOLERANCE);

    // 再转 1.5 s 到 225°，换算成 -135°
    assert(Feed(&imu, 91.0f, 300U));
    AssertNear(imu.yaw_deg, -135.0f, 0.05f);

    // 反向转回
    assert(Feed(&imu, -89.0f, 500U));
    AssertNear(imu.yaw_deg, 0.0f, 0.1f);
}

static void TestWrap(void)
{
    AssertNear(ImuYaw_Wrap(0.0f), 0.0f, TOLERANCE);
    AssertNear(ImuYaw_Wrap(190.0f), -170.0f, TOLERANCE);
    AssertNear(ImuYaw_Wrap(-190.0f), 170.0f, TOLERANCE);
    AssertNear(ImuYaw_Wrap(720.0f + 10.0f), 10.0f, TOLERANCE);
    AssertNear(ImuYaw_Wrap(NAN), 0.0f, TOLERANCE);
}

static void TestInvalid(void)
{
    ImuYaw_t imu;
    ImuYaw_Init(&imu);
    (void)Feed(&imu, 0.0f, IMU_BIAS_SAMPLES);
    ImuYaw_t saved = imu;

    // 非法输入：返回 false，状态不变
    assert(!ImuYaw_Update(&imu, NAN, DT_S));
    assert(!ImuYaw_Update(&imu, 1.0f, 0.0f));
    assert(!ImuYaw_Update(&imu, 1.0f, -DT_S));
    assert(!ImuYaw_Update(&imu, INFINITY, DT_S));
    assert(imu.yaw_deg == saved.yaw_deg && imu.calibrated);
    assert(!ImuYaw_Update(NULL, 1.0f, DT_S));
    ImuYaw_Init(NULL);
}

int main(void)
{
    TestBiasCalibration();
    TestMotionRestartsCalibration();
    TestIntegrationAndWrap();
    TestWrap();
    TestInvalid();

    puts("IMU yaw tests passed");
    return 0;
}
