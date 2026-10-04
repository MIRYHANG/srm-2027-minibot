//
// Created by YZH on 2026/10/4.
//
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "task_monitor.h"

static const uint32_t deadlines[TASK_ID_COUNT] = {
    [TASK_ID_PROTOCOL] = TASK_MONITOR_PROTOCOL_TIMEOUT_MS,
    [TASK_ID_ARM] = TASK_MONITOR_ARM_TIMEOUT_MS,
    [TASK_ID_SENSOR] = TASK_MONITOR_SENSOR_TIMEOUT_MS
};

/* 让其他任务都正常，仅保留待检查任务的旧心跳 */
static void BeatOthers(TaskId_t skipped, uint32_t now_ms)
{
    for (int idx = 0; idx < TASK_ID_COUNT; ++idx)
    {
        TaskId_t id = (TaskId_t)idx;

        if (id != skipped)
        {
            TaskMonitor_Beat(id, now_ms);
        }
    }
}

static void TestInitAlive(void)
{
    assert(TASK_MONITOR_PROTOCOL_TIMEOUT_MS == 30U);
    assert(TASK_MONITOR_ARM_TIMEOUT_MS == 100U);
    assert(TASK_MONITOR_SENSOR_TIMEOUT_MS == 400U);

    TaskMonitor_Init(1000U);
    assert(TaskMonitor_AllAlive(1000U));
}

/* 每个任务分别超时，即使其他任务正常也必须判定失败 */
static void TestEachTimeoutAndRecovery(void)
{
    for (int idx = 0; idx < TASK_ID_COUNT; ++idx)
    {
        TaskId_t id = (TaskId_t)idx;
        TaskMonitor_Init(1000U);

        uint32_t now_ms = 1000U + deadlines[idx] + 1U;
        BeatOthers(id, now_ms);

        assert(!TaskMonitor_AllAlive(now_ms));

        TaskMonitor_Beat(id, now_ms);
        assert(TaskMonitor_AllAlive(now_ms));
    }
}

static void TestExactDeadline(void)
{
    for (int idx = 0; idx < TASK_ID_COUNT; ++idx)
    {
        TaskId_t id = (TaskId_t)idx;
        TaskMonitor_Init(1000U);

        uint32_t boundary = 1000U + deadlines[idx];

        BeatOthers(id, boundary);
        assert(TaskMonitor_AllAlive(boundary));

        uint32_t expired = boundary + 1U;

        BeatOthers(id, expired);
        assert(!TaskMonitor_AllAlive(expired));
    }
}

static void TestWraparound(void)
{
    const uint32_t start = UINT32_C(0xFFFFFFF0);

    for (int idx = 0; idx < TASK_ID_COUNT; ++idx)
    {
        TaskId_t id = (TaskId_t)idx;
        TaskMonitor_Init(start);

        assert(TaskMonitor_AllAlive(start));

        // 所有期限都大于 16 ms，因此这里会跨过 UINT32_MAX
        uint32_t boundary = (uint32_t)(start + deadlines[idx]);
        assert(boundary < start);

        BeatOthers(id, boundary);
        assert(TaskMonitor_AllAlive(boundary));

        uint32_t expired = boundary + 1U;

        BeatOthers(id, expired);
        assert(!TaskMonitor_AllAlive(expired));

        TaskMonitor_Beat(id, expired);
        assert(TaskMonitor_AllAlive(expired));
    }
}

static void TestInvalidIdIgnored(void)
{
    static const TaskId_t invalid_ids[] = {
        (TaskId_t)-1,
        TASK_ID_COUNT,
        (TaskId_t)(TASK_ID_COUNT + 1),
        (TaskId_t)999
    };

    TaskMonitor_Init(1000U);

    // 非法 id 不得把任一合法任务的时间改成未来时间
    for (size_t idx = 0;
         idx < sizeof(invalid_ids) / sizeof(invalid_ids[0]);
         ++idx)
    {
        TaskMonitor_Beat(invalid_ids[idx], 2000U);
        assert(TaskMonitor_AllAlive(1000U));
    }

    // 非法 id 也不能误更新已经超时的 ProtocolTask
    const uint32_t now_ms =
        1000U + TASK_MONITOR_PROTOCOL_TIMEOUT_MS + 1U;

    BeatOthers(TASK_ID_PROTOCOL, now_ms);
    assert(!TaskMonitor_AllAlive(now_ms));

    for (size_t idx = 0;
         idx < sizeof(invalid_ids) / sizeof(invalid_ids[0]);
         ++idx)
    {
        TaskMonitor_Beat(invalid_ids[idx], now_ms);
        assert(!TaskMonitor_AllAlive(now_ms));
    }

    TaskMonitor_Beat(TASK_ID_PROTOCOL, now_ms);
    assert(TaskMonitor_AllAlive(now_ms));
}

int main(void)
{
    TestInitAlive();
    TestEachTimeoutAndRecovery();
    TestExactDeadline();
    TestWraparound();
    TestInvalidIdIgnored();

    puts("Task monitor tests passed");
    return 0;
}