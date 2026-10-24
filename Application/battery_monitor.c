//
// Created by YZH on 2026/10/24.
//

#include "battery_monitor.h"
#include <math.h>
#include <stddef.h>

typedef struct
{
    float cell_v;
    uint8_t percent;
} BatteryPoint_t;

// 锂电池单节静置电压与剩余电量的近似关系，电压从低到高排列
static const BatteryPoint_t BATTERY_CURVE[] = {
    {3.30f, 0U},
    {3.60f, 5U},
    {3.70f, 15U},
    {3.75f, 25U},
    {3.80f, 40U},
    {3.85f, 55U},
    {3.95f, 70U},
    {4.05f, 85U},
    {4.20f, 100U},
};

#define CURVE_LEN (sizeof(BATTERY_CURVE) / sizeof(BATTERY_CURVE[0]))

void BatteryMonitor_Init(BatteryMonitor_t *monitor)
{
    if (monitor == NULL)
    {
        return;
    }

    *monitor = (BatteryMonitor_t){0};
}

/**
 * @brief 按当前状态带回差地判断电压落在哪一档
 * @note 已经处于某一档时，要高出阈值 BATTERY_HYSTERESIS_CELL_V 才回到上一档
 */
static BatteryLevel_t Classify(float pack_v, BatteryLevel_t current)
{
    float critical_v = BATTERY_CRITICAL_CELL_V * (float)BATTERY_CELLS;
    float low_v = BATTERY_LOW_CELL_V * (float)BATTERY_CELLS;
    float hysteresis_v = BATTERY_HYSTERESIS_CELL_V * (float)BATTERY_CELLS;

    if (current == BATTERY_CRITICAL)
    {
        critical_v += hysteresis_v;
    }
    if (current == BATTERY_CRITICAL || current == BATTERY_LOW)
    {
        low_v += hysteresis_v;
    }

    if (pack_v < critical_v)
    {
        return BATTERY_CRITICAL;
    }
    if (pack_v < low_v)
    {
        return BATTERY_LOW;
    }
    return BATTERY_OK;
}

BatteryLevel_t BatteryMonitor_Update(BatteryMonitor_t *monitor, float bus_v,
                                     bool valid, float dt_s)
{
    if (monitor == NULL || !isfinite(dt_s) || dt_s <= 0.0f)
    {
        return BATTERY_UNKNOWN;
    }

    if (!valid || !isfinite(bus_v) || bus_v < 0.0f)
    {
        BatteryMonitor_Init(monitor);
        return BATTERY_UNKNOWN;
    }

    // 第一次有效读数：直接作为滤波初值，立即判定
    if (monitor->level == BATTERY_UNKNOWN)
    {
        monitor->filtered_v = bus_v;
        monitor->level = Classify(bus_v, BATTERY_UNKNOWN);
        monitor->pending = monitor->level;
        monitor->pending_s = 0.0f;
        return monitor->level;
    }

    float alpha = dt_s / (BATTERY_FILTER_TAU_S + dt_s);
    monitor->filtered_v += alpha * (bus_v - monitor->filtered_v);

    BatteryLevel_t next = Classify(monitor->filtered_v, monitor->level);

    if (next == monitor->level)
    {
        monitor->pending = next;
        monitor->pending_s = 0.0f;
        return monitor->level;
    }

    // 目标档位变了就重新计时，避免在两档之间来回抖动时累计时间
    if (next != monitor->pending)
    {
        monitor->pending = next;
        monitor->pending_s = 0.0f;
    }

    monitor->pending_s += dt_s;
    if (monitor->pending_s >= BATTERY_CONFIRM_S)
    {
        monitor->level = next;
        monitor->pending_s = 0.0f;
    }

    return monitor->level;
}

uint8_t BatteryMonitor_Percent(float pack_v)
{
    if (!isfinite(pack_v))
    {
        return 0U;
    }

    float cell_v = pack_v / (float)BATTERY_CELLS;

    if (cell_v <= BATTERY_CURVE[0].cell_v)
    {
        return BATTERY_CURVE[0].percent;
    }

    for (size_t idx = 1U; idx < CURVE_LEN; idx++)
    {
        const BatteryPoint_t *lo = &BATTERY_CURVE[idx - 1U];
        const BatteryPoint_t *hi = &BATTERY_CURVE[idx];

        if (cell_v <= hi->cell_v)
        {
            float ratio = (cell_v - lo->cell_v) / (hi->cell_v - lo->cell_v);
            float percent = (float)lo->percent + ratio * (float)(hi->percent - lo->percent);
            return (uint8_t)lroundf(percent);
        }
    }

    return BATTERY_CURVE[CURVE_LEN - 1U].percent;
}
