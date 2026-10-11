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

#define RAMP_DT_S 0.1f


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

static MecanumWheelRpm_t Wheels(float fl, float fr, float rl, float rr)
{
    return (MecanumWheelRpm_t){.fl = fl, .fr = fr, .rl = rl, .rr = rr};
}

static void TestRampAccelAndDecel(void)
{
    const float step = CHASSIS_WHEEL_RPM_PER_S * RAMP_DT_S;
    const float goal = 5.5f * step; // 不是整数步，检查最后一步刚好到达、不冲过头
    ChassisRamp_t ramp;
    MecanumWheelRpm_t out;
    MecanumWheelRpm_t target = Wheels(goal, goal, goal, goal);

    ChassisRamp_Reset(&ramp);

    // 加速：每步只增加 step
    for (int count = 1; count <= 5; count++)
    {
        assert(ChassisRamp_Step(&ramp, &target, RAMP_DT_S, &out));
        float expected = (float)count * step;
        AssertWheels(&out, expected, expected, expected, expected);
    }

    // 剩下半步：直接到达目标，之后保持不动
    assert(ChassisRamp_Step(&ramp, &target, RAMP_DT_S, &out));
    AssertWheels(&out, goal, goal, goal, goal);
    assert(ChassisRamp_Step(&ramp, &target, RAMP_DT_S, &out));
    AssertWheels(&out, goal, goal, goal, goal);

    // 松开摇杆：减速同样受限制，不会一下停住
    MecanumWheelRpm_t zero = Wheels(0.0f, 0.0f, 0.0f, 0.0f);
    assert(ChassisRamp_Step(&ramp, &zero, RAMP_DT_S, &out));
    float slowed = goal - step;
    AssertWheels(&out, slowed, slowed, slowed, slowed);
}

static void TestRampKeepsDirection(void)
{
    const float step = CHASSIS_WHEEL_RPM_PER_S * RAMP_DT_S;
    ChassisRamp_t ramp;
    MecanumWheelRpm_t out;

    // 斜着推满：(0, 30, 30, 0) 的比例在过渡过程中保持不变
    ChassisRamp_Reset(&ramp);
    MecanumWheelRpm_t target = Wheels(0.0f, 30.0f, 30.0f, 0.0f);
    assert(ChassisRamp_Step(&ramp, &target, RAMP_DT_S, &out));
    AssertWheels(&out, 0.0f, step, step, 0.0f);

    // 变化量不同的四个轮子：变化最大的走满一步，其他按比例走
    ChassisRamp_Reset(&ramp);
    target = Wheels(-10.0f * step, 20.0f * step, 5.0f * step, 0.0f);
    assert(ChassisRamp_Step(&ramp, &target, RAMP_DT_S, &out));
    AssertWheels(&out, -0.5f * step, step, 0.25f * step, 0.0f);
}

static void TestRampSmallChangeAndAlias(void)
{
    const float step = CHASSIS_WHEEL_RPM_PER_S * RAMP_DT_S;
    ChassisRamp_t ramp;
    ChassisRamp_Reset(&ramp);

    // 变化量小于一步：直接到达；out 和 target 可以是同一个对象
    MecanumWheelRpm_t wheels = Wheels(0.5f * step, -0.5f * step, 0.25f * step, 0.0f);
    assert(ChassisRamp_Step(&ramp, &wheels, RAMP_DT_S, &wheels));
    AssertWheels(&wheels, 0.5f * step, -0.5f * step, 0.25f * step, 0.0f);
}

static void TestRampInvalid(void)
{
    const float step = CHASSIS_WHEEL_RPM_PER_S * RAMP_DT_S;
    ChassisRamp_t ramp;
    MecanumWheelRpm_t out;
    MecanumWheelRpm_t target = Wheels(10.0f * step, 10.0f * step, 10.0f * step, 10.0f * step);

    ChassisRamp_Reset(&ramp);
    assert(ChassisRamp_Step(&ramp, &target, RAMP_DT_S, &out));

    // 非法参数：返回 false，输出上一次的转速，ramp 不变
    ChassisRamp_t saved = ramp;
    static const float bad_dt[] = {0.0f, -0.1f, NAN, INFINITY};
    for (size_t idx = 0U; idx < sizeof(bad_dt) / sizeof(bad_dt[0]); idx++)
    {
        memset(&out, 0xA5, sizeof(out));
        assert(!ChassisRamp_Step(&ramp, &target, bad_dt[idx], &out));
        AssertWheels(&out, step, step, step, step);
        assert(memcmp(&ramp, &saved, sizeof(ramp)) == 0);
    }

    MecanumWheelRpm_t bad_target = Wheels(NAN, 0.0f, 0.0f, 0.0f);
    assert(!ChassisRamp_Step(&ramp, &bad_target, RAMP_DT_S, &out));
    AssertWheels(&out, step, step, step, step);
    assert(!ChassisRamp_Step(&ramp, NULL, RAMP_DT_S, &out));
    assert(!ChassisRamp_Step(&ramp, &target, RAMP_DT_S, NULL));
    assert(!ChassisRamp_Step(NULL, &target, RAMP_DT_S, &out));
    assert(memcmp(&ramp, &saved, sizeof(ramp)) == 0);

    // 急停后清零：下一次从 0 开始加速
    ChassisRamp_Reset(&ramp);
    assert(ChassisRamp_Step(&ramp, &target, RAMP_DT_S, &out));
    AssertWheels(&out, step, step, step, step);
    ChassisRamp_Reset(NULL);
}

int main(void)
{
    TestGearTable();
    TestFullForwardEqualsGearRpm();
    TestDirections();
    TestNeverExceedsGear();
    TestDisabledAndInvalid();
    TestRampAccelAndDecel();
    TestRampKeepsDirection();
    TestRampSmallChangeAndAlias();
    TestRampInvalid();

    puts("Chassis drive tests passed");
    return 0;
}