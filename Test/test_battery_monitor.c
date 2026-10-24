//
// Created by YZH on 2026/10/24.
//
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "battery_monitor.h"

#define DT_S 0.1f
#define PACK(cell_v) ((cell_v) * (float)BATTERY_CELLS)

/* 以固定电压连续输入 seconds 秒，返回最后的状态 */
static BatteryLevel_t Feed(BatteryMonitor_t *m, float pack_v, float seconds)
{
    BatteryLevel_t level = BATTERY_UNKNOWN;
    int steps = (int)lroundf(seconds / DT_S);
    for (int idx = 0; idx < steps; idx++)
    {
        level = BatteryMonitor_Update(m, pack_v, true, DT_S);
    }
    return level;
}

static void TestFirstReadingClassifiedImmediately(void)
{
    BatteryMonitor_t m;

    BatteryMonitor_Init(&m);
    assert(BatteryMonitor_Update(&m, 12.4f, true, DT_S) == BATTERY_OK);

    BatteryMonitor_Init(&m);
    assert(BatteryMonitor_Update(&m, PACK(3.40f), true, DT_S) == BATTERY_LOW);

    BatteryMonitor_Init(&m);
    assert(BatteryMonitor_Update(&m, PACK(3.20f), true, DT_S) == BATTERY_CRITICAL);
}

static void TestShortDipIgnored(void)
{
    BatteryMonitor_t m;
    BatteryMonitor_Init(&m);
    assert(BatteryMonitor_Update(&m, 12.0f, true, DT_S) == BATTERY_OK);

    // 电机启动时拉到 9 V 只有 0.3 s：滤波后不到阈值，状态不变
    assert(Feed(&m, 9.0f, 0.3f) == BATTERY_OK);
    assert(Feed(&m, 12.0f, 3.0f) == BATTERY_OK);
}

static void TestDropNeedsConfirm(void)
{
    BatteryMonitor_t m;
    BatteryMonitor_Init(&m);
    assert(BatteryMonitor_Update(&m, 12.0f, true, DT_S) == BATTERY_OK);

    // 电压持续偏低：滤波追上后还要再确认 2 s 才报 LOW
    assert(Feed(&m, PACK(3.40f), 2.0f) == BATTERY_OK);
    assert(Feed(&m, PACK(3.40f), 4.0f) == BATTERY_LOW);

    // 继续下降到欠压
    assert(Feed(&m, PACK(3.20f), 1.0f) == BATTERY_LOW);
    assert(Feed(&m, PACK(3.20f), 5.0f) == BATTERY_CRITICAL);
}

static void TestHysteresis(void)
{
    BatteryMonitor_t m;
    BatteryMonitor_Init(&m);
    assert(BatteryMonitor_Update(&m, PACK(3.20f), true, DT_S) == BATTERY_CRITICAL);

    // 刚回到阈值以上，没超过回差，仍然欠压
    assert(Feed(&m, PACK(3.35f), 10.0f) == BATTERY_CRITICAL);

    // 超过阈值加回差，确认后回到 LOW；还要再高过 LOW 的回差才回到 OK
    assert(Feed(&m, PACK(3.45f), 10.0f) == BATTERY_LOW);
    assert(Feed(&m, PACK(3.55f), 10.0f) == BATTERY_LOW);
    assert(Feed(&m, PACK(3.65f), 10.0f) == BATTERY_OK);
}

static void TestInvalidReading(void)
{
    BatteryMonitor_t m;
    BatteryMonitor_Init(&m);
    assert(BatteryMonitor_Update(&m, PACK(3.20f), true, DT_S) == BATTERY_CRITICAL);

    // 传感器故障：状态回到未知，不当作欠压
    assert(BatteryMonitor_Update(&m, 0.0f, false, DT_S) == BATTERY_UNKNOWN);
    assert(BatteryMonitor_Update(&m, NAN, true, DT_S) == BATTERY_UNKNOWN);

    // 恢复后重新从第一次读数开始
    assert(BatteryMonitor_Update(&m, 12.0f, true, DT_S) == BATTERY_OK);

    // 非法参数：返回未知，状态不变
    BatteryMonitor_t saved = m;
    assert(BatteryMonitor_Update(&m, 12.0f, true, 0.0f) == BATTERY_UNKNOWN);
    assert(BatteryMonitor_Update(&m, 12.0f, true, NAN) == BATTERY_UNKNOWN);
    assert(m.level == saved.level && m.filtered_v == saved.filtered_v);
    assert(BatteryMonitor_Update(NULL, 12.0f, true, DT_S) == BATTERY_UNKNOWN);
    BatteryMonitor_Init(NULL);
}

static void TestPercent(void)
{
    assert(BatteryMonitor_Percent(PACK(4.20f)) == 100U);
    assert(BatteryMonitor_Percent(13.0f) == 100U);
    assert(BatteryMonitor_Percent(PACK(3.30f)) == 0U);
    assert(BatteryMonitor_Percent(8.0f) == 0U);
    assert(BatteryMonitor_Percent(PACK(3.80f)) == 40U);
    assert(BatteryMonitor_Percent(PACK(3.775f)) == 33U); // 3.75～3.80 之间线性插值
    assert(BatteryMonitor_Percent(NAN) == 0U);

    // 电压越高电量越多
    uint8_t last = 0U;
    for (float v = 9.0f; v <= 13.0f; v += 0.05f)
    {
        uint8_t p = BatteryMonitor_Percent(v);
        assert(p >= last);
        last = p;
    }
}

int main(void)
{
    TestFirstReadingClassifiedImmediately();
    TestShortDipIgnored();
    TestDropNeedsConfirm();
    TestHysteresis();
    TestInvalidReading();
    TestPercent();

    puts("Battery monitor tests passed");
    return 0;
}
