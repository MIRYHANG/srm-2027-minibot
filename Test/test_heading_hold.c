//
// Created by YZH on 2026/10/24.
//
// 编译时传入 -DHEADING_KP_RPM_PER_DEG=1.0f -DHEADING_KD_RPM_PER_DPS=0.1f
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "chassis_drive.h"
#include "heading_hold.h"

#define DT_S 0.01f
#define TOLERANCE 0.0001f

static void AssertNear(float actual, float expected)
{
    assert(fabsf(actual - expected) < TOLERANCE);
}

static ChassisCmd_t Cmd(float forward, float left, float turn, uint8_t gear)
{
    return (ChassisCmd_t){.enabled = true, .forward = forward, .left = left,
                          .turn = turn, .gear = gear};
}

/* 期望的修正量：rpm 换算成相对挡位满速的比例 */
static float Expected(float error_deg, float rate_dps, uint8_t gear)
{
    float rpm = HEADING_KP_RPM_PER_DEG * error_deg - HEADING_KD_RPM_PER_DPS * rate_dps;
    if (rpm > HEADING_MAX_RPM)
    {
        rpm = HEADING_MAX_RPM;
    }
    if (rpm < -HEADING_MAX_RPM)
    {
        rpm = -HEADING_MAX_RPM;
    }
    return rpm / (ChassisDrive_GearMaxRpm(gear) * CHASSIS_TURN_SCALE);
}

static void TestLocksAndCorrectsDrift(void)
{
    HeadingHold_t hold;
    HeadingHold_Reset(&hold);
    ChassisCmd_t cmd = Cmd(1.0f, 0.0f, 0.0f, 2U);

    // 从停车直接前进：立即锁定当前航向，没有偏差时不修正
    AssertNear(HeadingHold_Update(&hold, &cmd, true, 30.0f, 0.0f, DT_S), 0.0f);
    assert(hold.locked);
    AssertNear(hold.target_deg, 30.0f);

    // 车往顺时针偏 4°：输出逆时针修正（正）
    float out = HeadingHold_Update(&hold, &cmd, true, 26.0f, 0.0f, DT_S);
    assert(out > 0.0f);
    AssertNear(out, Expected(4.0f, 0.0f, 2U));

    // 往逆时针偏：输出顺时针修正（负），角速度项抑制摆动
    out = HeadingHold_Update(&hold, &cmd, true, 33.0f, 5.0f, DT_S);
    assert(out < 0.0f);
    AssertNear(out, Expected(-3.0f, 5.0f, 2U));
}

static void TestCorrectionIndependentOfGear(void)
{
    HeadingHold_t hold;
    HeadingHold_Reset(&hold);
    ChassisCmd_t cmd = Cmd(1.0f, 0.0f, 0.0f, 1U);
    (void)HeadingHold_Update(&hold, &cmd, true, 0.0f, 0.0f, DT_S);

    // 同样偏差，换算回 rpm 后各挡相同
    for (uint8_t gear = 1U; gear <= 3U; gear++)
    {
        cmd.gear = gear;
        float out = HeadingHold_Update(&hold, &cmd, true, -2.0f, 0.0f, DT_S);
        float rpm = out * ChassisDrive_GearMaxRpm(gear) * CHASSIS_TURN_SCALE;
        AssertNear(rpm, HEADING_KP_RPM_PER_DEG * 2.0f);
    }
}

static void TestClampAndWrap(void)
{
    HeadingHold_t hold;
    HeadingHold_Reset(&hold);
    ChassisCmd_t cmd = Cmd(0.0f, 1.0f, 0.0f, 3U);
    (void)HeadingHold_Update(&hold, &cmd, true, 170.0f, 0.0f, DT_S);

    // 跨过 ±180°：从 170° 偏到 -175°，实际是逆时针偏了 15°，输出顺时针修正
    float out = HeadingHold_Update(&hold, &cmd, true, -175.0f, 0.0f, DT_S);
    assert(out < 0.0f);
    AssertNear(out, Expected(-15.0f, 0.0f, 3U));

    // 偏差很大时不超过上限
    out = HeadingHold_Update(&hold, &cmd, true, 80.0f, 0.0f, DT_S);
    AssertNear(out, HEADING_MAX_RPM / (ChassisDrive_GearMaxRpm(3U) * CHASSIS_TURN_SCALE));
}

static void TestOperatorTurnAndSettle(void)
{
    HeadingHold_t hold;
    HeadingHold_Reset(&hold);
    ChassisCmd_t cmd = Cmd(1.0f, 0.0f, 0.0f, 2U);
    (void)HeadingHold_Update(&hold, &cmd, true, 0.0f, 0.0f, DT_S);

    // 按住自转键：原样输出 turn，解除锁定
    cmd.turn = -1.0f;
    AssertNear(HeadingHold_Update(&hold, &cmd, true, 10.0f, -60.0f, DT_S), -1.0f);
    assert(!hold.locked);

    // 松开后等待 HEADING_SETTLE_S，期间不修正
    cmd.turn = 0.0f;
    int settle_steps = (int)lroundf(HEADING_SETTLE_S / DT_S);
    for (int idx = 0; idx < settle_steps - 1; idx++)
    {
        AssertNear(HeadingHold_Update(&hold, &cmd, true, 20.0f + (float)idx, 5.0f, DT_S), 0.0f);
        assert(!hold.locked);
    }

    // 时间到：锁定此刻的航向，而不是按自转键之前的航向
    (void)HeadingHold_Update(&hold, &cmd, true, 50.0f, 0.0f, DT_S);
    (void)HeadingHold_Update(&hold, &cmd, true, 50.0f, 0.0f, DT_S);
    assert(hold.locked);
    AssertNear(hold.target_deg, 50.0f);
}

static void TestNoCorrectionWhenStoppedOrInvalid(void)
{
    HeadingHold_t hold;
    HeadingHold_Reset(&hold);
    ChassisCmd_t cmd = Cmd(1.0f, 0.0f, 0.0f, 2U);
    (void)HeadingHold_Update(&hold, &cmd, true, 0.0f, 0.0f, DT_S);

    // 摇杆回中：不修正，解除锁定，车被推动也不会扭回去
    cmd.forward = 0.02f;
    AssertNear(HeadingHold_Update(&hold, &cmd, true, 45.0f, 0.0f, DT_S), 0.0f);
    assert(!hold.locked);

    // 再次开动：锁定新的航向
    cmd.forward = 1.0f;
    AssertNear(HeadingHold_Update(&hold, &cmd, true, 45.0f, 0.0f, DT_S), 0.0f);
    AssertNear(hold.target_deg, 45.0f);

    // IMU 无效：不修正
    AssertNear(HeadingHold_Update(&hold, &cmd, false, 0.0f, 0.0f, DT_S), 0.0f);
    assert(!hold.locked);
    AssertNear(HeadingHold_Update(&hold, &cmd, true, NAN, 0.0f, DT_S), 0.0f);

    // 未使能：输出 0 并清除状态
    (void)HeadingHold_Update(&hold, &cmd, true, 0.0f, 0.0f, DT_S);
    cmd.enabled = false;
    AssertNear(HeadingHold_Update(&hold, &cmd, true, 10.0f, 0.0f, DT_S), 0.0f);
    assert(!hold.locked);

    // 空指针
    AssertNear(HeadingHold_Update(NULL, &cmd, true, 0.0f, 0.0f, DT_S), 0.0f);
    AssertNear(HeadingHold_Update(&hold, NULL, true, 0.0f, 0.0f, DT_S), 0.0f);
    HeadingHold_Reset(NULL);
}

int main(void)
{
    assert(HEADING_KP_RPM_PER_DEG > 0.0f); // 必须用非零增益编译，否则测不出方向

    TestLocksAndCorrectsDrift();
    TestCorrectionIndependentOfGear();
    TestClampAndWrap();
    TestOperatorTurnAndSettle();
    TestNoCorrectionWhenStoppedOrInvalid();

    puts("Heading hold tests passed");
    return 0;
}
