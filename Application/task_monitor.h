//
// Created by YZH on 2026/10/4.
//

#ifndef XIAOSAI_TASK_MONITOR_H
#define XIAOSAI_TASK_MONITOR_H
#include <stdbool.h>
#include <stdint.h>

#define TASK_MONITOR_PROTOCOL_TIMEOUT_MS 30U
#define TASK_MONITOR_ARM_TIMEOUT_MS      100U
#define TASK_MONITOR_SENSOR_TIMEOUT_MS   400U

typedef enum
{
    TASK_ID_PROTOCOL = 0,
    TASK_ID_ARM,
    TASK_ID_SENSOR,
    TASK_ID_COUNT
} TaskId_t;

/**
 * @brief 初始化所有任务心跳，以当前时间作为启动宽限的起点
 * @note 仅在调度器启动前调用，不在运行过程中重复初始化
 */
void TaskMonitor_Init(uint32_t now_ms);

/**
 * @brief 记录指定任务完成一次周期处理的时间
 * @note 由对应任务调用，非法 id 被忽略，不在中断中调用
 */
void TaskMonitor_Beat(TaskId_t id, uint32_t now_ms);

/**
 * @brief 检查全部任务是否仍在各自的心跳期限内
 * @return 全部未超时返回 true，任一超时返回 false
 * @note now_ms 与心跳时间必须使用同一毫秒时基
 */
bool TaskMonitor_AllAlive(uint32_t now_ms);

#endif //XIAOSAI_TASK_MONITOR_H
