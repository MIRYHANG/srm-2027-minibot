//
// Created by YZH on 2026/10/5.
//
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "power_display.h"

static PowerDisplayLine_t lines[POWER_DISPLAY_LINES];

static void Format(float v, float i, float p, bool valid, ResetCause_t cause)
{
    Ina226Reading_t reading = {.bus_v = v, .current_a = i, .power_w = p};

    // 先填满垃圾，确认每一行都被重写并正确结尾
    memset(lines, 'Z', sizeof(lines));
    PowerDisplay_Format(&reading, valid, cause, lines);

    for (size_t idx = 0U; idx < POWER_DISPLAY_LINES; idx++)
    {
        assert(strlen(lines[idx]) <= POWER_DISPLAY_COLS);
    }
}

static void TestValidReading(void)
{
    Format(12.34f, 1.25f, 15.42f, true, RESET_CAUSE_POWER);

    assert(strcmp(lines[0], "XIAOSAI POWER") == 0);
    assert(strcmp(lines[1], "") == 0);
    assert(strcmp(lines[2], "U   12.34 V") == 0);
    assert(strcmp(lines[3], "I    1.25 A") == 0);
    assert(strcmp(lines[4], "P    15.4 W") == 0);
    assert(strcmp(lines[5], "") == 0);
    assert(strcmp(lines[6], "INA226 OK") == 0);
    assert(strcmp(lines[7], "RST POWER") == 0);
}

static void TestRoundingAndSign(void)
{
    // 四舍五入、进位、负数
    Format(0.0f, -1.25f, -1677.7f, true, RESET_CAUSE_POWER);
    assert(strcmp(lines[2], "U    0.00 V") == 0);
    assert(strcmp(lines[3], "I   -1.25 A") == 0);
    assert(strcmp(lines[4], "P -1677.7 W") == 0);

    Format(9.996f, 0.004f, 99.96f, true, RESET_CAUSE_POWER);
    assert(strcmp(lines[2], "U   10.00 V") == 0);
    assert(strcmp(lines[3], "I    0.00 A") == 0);
    assert(strcmp(lines[4], "P   100.0 W") == 0);

    // 四舍五入后为 0 的负数不显示负号
    Format(40.96f, -0.004f, -0.04f, true, RESET_CAUSE_POWER);
    assert(strcmp(lines[2], "U   40.96 V") == 0);
    assert(strcmp(lines[3], "I    0.00 A") == 0);
    assert(strcmp(lines[4], "P     0.0 W") == 0);
}

static void TestInvalidValues(void)
{
    // 读数无效：数值显示横线
    Format(12.34f, 1.25f, 15.42f, false, RESET_CAUSE_IWDG);
    assert(strcmp(lines[2], "U   --.-- V") == 0);
    assert(strcmp(lines[3], "I   --.-- A") == 0);
    assert(strcmp(lines[4], "P    --.- W") == 0);
    assert(strcmp(lines[6], "INA226 ERROR") == 0);
    assert(strcmp(lines[7], "RST IWDG") == 0);

    // 单个数值异常：只有这一项显示横线
    Format(NAN, INFINITY, 12345678.0f, true, RESET_CAUSE_POWER);
    assert(strcmp(lines[2], "U   --.-- V") == 0);
    assert(strcmp(lines[3], "I   --.-- A") == 0);
    assert(strcmp(lines[4], "P    --.- W") == 0);
    assert(strcmp(lines[6], "INA226 OK") == 0);

    // 放不进 7 个字符：显示横线
    Format(-99999.9f, 0.0f, 0.0f, true, RESET_CAUSE_POWER);
    assert(strcmp(lines[2], "U   --.-- V") == 0);

    // 数值本身不大，但加上负号后是 8 个字符，也放不下
    Format(-9999.99f, 9999.99f, 0.0f, true, RESET_CAUSE_POWER);
    assert(strcmp(lines[2], "U   --.-- V") == 0);
    assert(strcmp(lines[3], "I 9999.99 A") == 0);

    // reading 为 NULL 视为无效
    memset(lines, 'Z', sizeof(lines));
    PowerDisplay_Format(NULL, true, RESET_CAUSE_PIN, lines);
    assert(strcmp(lines[2], "U   --.-- V") == 0);
    assert(strcmp(lines[6], "INA226 ERROR") == 0);
    assert(strcmp(lines[7], "RST PIN") == 0);

    PowerDisplay_Format(NULL, false, RESET_CAUSE_PIN, NULL);
}

static void TestResetCauses(void)
{
    static const struct
    {
        ResetCause_t cause;
        const char *text;
    } cases[] = {
        {RESET_CAUSE_UNKNOWN, "RST ?"},
        {RESET_CAUSE_POWER, "RST POWER"},
        {RESET_CAUSE_PIN, "RST PIN"},
        {RESET_CAUSE_SOFTWARE, "RST SOFT"},
        {RESET_CAUSE_IWDG, "RST IWDG"},
        {RESET_CAUSE_WWDG, "RST WWDG"},
        {RESET_CAUSE_LOW_POWER, "RST LOWPWR"},
        {(ResetCause_t)99, "RST ?"},
    };

    for (size_t idx = 0U; idx < sizeof(cases) / sizeof(cases[0]); idx++)
    {
        Format(12.0f, 1.0f, 12.0f, true, cases[idx].cause);
        assert(strcmp(lines[7], cases[idx].text) == 0);
    }
}

int main(void)
{
    TestValidReading();
    TestRoundingAndSign();
    TestInvalidValues();
    TestResetCauses();

    puts("Power display tests passed");
    return 0;
}
