//
// Created by YZH on 2026/10/9.
//
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "chassis_drive.h"

#define TOLERANCE 0.0001f

static void AssertNear(float actual, float expected)
{
    assert(fabsf(actual - expected) < TOLERANCE);
}

static void AssertWheels(const MecanumWheelRpm_t *w, float fl, float fr, float rl, float rr)
{
    AssertNear(w->fl, fl);
    AssertNear(w->fr, fr);
    AssertNear(w->rl, rl);
    AssertNear(w->rr, rr);
}

static float Peak(const MecanumWheelRpm_t *w)
{
    return fmaxf(fmaxf(fabsf(w->fl), fabsf(w->fr)), fmaxf(fabsf(w->rl), fabsf(w->rr)));
}

static ChassisCmd_t Cmd(float forward, float left, float turn, uint8_t gear)
{
    ChassisCmd_t cmd = {0};
    cmd.enabled = true;
    cmd.forward = forward;
    cmd.left = left;
    cmd.turn = turn;
    cmd.gear = gear;
    return cmd;
}

/* 把 out 填满垃圾后调用，确认每次都会被重写 */
static bool Convert(const ChassisCmd_t *cmd, MecanumWheelRpm_t *out)
{
    memset(out, 0xA5, sizeof(*out));
    return ChassisDrive_ToWheelRpm(cmd, out);
}

static void TestGearTable(void)
{
    // 挡位越高越快，非法挡位为 0
    assert(ChassisDrive_GearMaxRpm(1U) > 0.0f);
    assert(ChassisDrive_GearMaxRpm(2U) > ChassisDrive_GearMaxRpm(1U));
    assert(ChassisDrive_GearMaxRpm(3U) > ChassisDrive_GearMaxRpm(2U));
    assert(ChassisDrive_GearMaxRpm(0U) == 0.0f);
    assert(ChassisDrive_GearMaxRpm(4U) == 0.0f);
}

static void TestFullForwardEqualsGearRpm(void)
{
    MecanumWheelRpm_t w;

    for (uint8_t gear = 1U; gear <= 3U; gear++)
    {
        float max_rpm = ChassisDrive_GearMaxRpm(gear);

        // 推满：四轮都等于挡位转速
        ChassisCmd_t cmd = Cmd(1.0f, 0.0f, 0.0f, gear);
        assert(Convert(&cmd, &w));
        AssertWheels(&w, max_rpm, max_rpm, max_rpm, max_rpm);

        // 推一半：一半
        cmd = Cmd(-0.5f, 0.0f, 0.0f, gear);
        assert(Convert(&cmd, &w));
        float half = -0.5f * max_rpm;
        AssertWheels(&w, half, half, half, half);
    }
}

static void TestDirections(void)
{
    MecanumWheelRpm_t w;
    float max_rpm = ChassisDrive_GearMaxRpm(2U);

    // 向左横移
    ChassisCmd_t cmd = Cmd(0.0f, 1.0f, 0.0f, 2U);
    assert(Convert(&cmd, &w));
    AssertWheels(&w, -max_rpm, max_rpm, max_rpm, -max_rpm);

    // 逆时针自转，乘以自转系数
    float turn_rpm = max_rpm * CHASSIS_TURN_SCALE;
    cmd = Cmd(0.0f, 0.0f, 1.0f, 2U);
    assert(Convert(&cmd, &w));
    AssertWheels(&w, -turn_rpm, turn_rpm, -turn_rpm, turn_rpm);

    // 斜着推满：最快的轮子正好等于挡位转速，方向不变
    cmd = Cmd(1.0f, 1.0f, 0.0f, 2U);
    assert(Convert(&cmd, &w));
    AssertWheels(&w, 0.0f, max_rpm, max_rpm, 0.0f);
}

static void TestNeverExceedsGear(void)
{
    static const float values[] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};
    const size_t count = sizeof(values) / sizeof(values[0]);
    MecanumWheelRpm_t w;

    // 所有组合下都不超过挡位转速
    for (uint8_t gear = 1U; gear <= 3U; gear++)
    {
        for (size_t f = 0U; f < count; f++)
        {
            for (size_t l = 0U; l < count; l++)
            {
                for (size_t t = 0U; t < count; t++)
                {
                    ChassisCmd_t cmd = Cmd(values[f], values[l], values[t], gear);
                    assert(Convert(&cmd, &w));
                    assert(Peak(&w) <= ChassisDrive_GearMaxRpm(gear) + TOLERANCE);
                }
            }
        }
    }
}

static void TestDisabledAndInvalid(void)
{
    MecanumWheelRpm_t w;

    // 未使能：返回 true，四轮为零
    ChassisCmd_t cmd = Cmd(1.0f, 1.0f, 1.0f, 3U);
    cmd.enabled = false;
    assert(Convert(&cmd, &w));
    AssertWheels(&w, 0.0f, 0.0f, 0.0f, 0.0f);

    // 非法挡位、越界或 NaN 的输入：返回 false，四轮为零
    static const ChassisCmd_t bad[] = {
        {.enabled = true, .forward = 1.0f, .gear = 0U},
        {.enabled = true, .forward = 1.0f, .gear = 4U},
        {.enabled = true, .forward = 1.5f, .gear = 1U},
        {.enabled = true, .left = -1.01f, .gear = 1U},
        {.enabled = true, .turn = NAN, .gear = 1U},
        {.enabled = true, .forward = INFINITY, .gear = 1U},
    };

    for (size_t idx = 0U; idx < sizeof(bad) / sizeof(bad[0]); idx++)
    {
        assert(!Convert(&bad[idx], &w));
        AssertWheels(&w, 0.0f, 0.0f, 0.0f, 0.0f);
    }

    assert(!Convert(NULL, &w));
    AssertWheels(&w, 0.0f, 0.0f, 0.0f, 0.0f);
    assert(!ChassisDrive_ToWheelRpm(&cmd, NULL));
}

int main(void)
{
    TestGearTable();
    TestFullForwardEqualsGearRpm();
    TestDirections();
    TestNeverExceedsGear();
    TestDisabledAndInvalid();

    puts("Chassis drive tests passed");
    return 0;
}