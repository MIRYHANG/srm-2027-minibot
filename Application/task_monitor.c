//
// Created by YZH on 2026/10/4.
//

#include "task_monitor.h"

static volatile uint32_t last_beat_ms[TASK_ID_COUNT];

static const uint32_t timeout_ms[TASK_ID_COUNT] = {
    [TASK_ID_PROTOCOL] = TASK_MONITOR_PROTOCOL_TIMEOUT_MS,
    [TASK_ID_SENSOR] = TASK_MONITOR_SENSOR_TIMEOUT_MS,
    [TASK_ID_ARM] = TASK_MONITOR_ARM_TIMEOUT_MS
};

void TaskMonitor_Init(uint32_t now_ms)
{
    for (int idx = 0;idx < TASK_ID_COUNT;idx++)
    {
        last_beat_ms[idx] = now_ms;
    }
}

void TaskMonitor_Beat(TaskId_t id, uint32_t now_ms)
{
    // 转为无符号数后，负值和超出上限的值都能被拒绝
    if ((uint32_t)id >= (uint32_t)TASK_ID_COUNT)
    {
        return;
    }

    last_beat_ms[id] = now_ms;
}

bool TaskMonitor_AllAlive(uint32_t now_ms)
{
    for (int idx = 0; idx < TASK_ID_COUNT; ++idx)
    {
        // 每个 volatile 元素只读取一次
        uint32_t last = last_beat_ms[idx];

        // 无符号减法允许计时器从 UINT32_MAX 回绕到零
        // 刚好等于期限仍有效，超过期限才判定超时
        if ((uint32_t)(now_ms - last) > timeout_ms[idx])
        {
            return false;
        }
    }

    return true;
}