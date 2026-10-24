//
// Created by YZH on 2026/10/24.
//

#ifndef XIAOSAI_BATTERY_MONITOR_H
#define XIAOSAI_BATTERY_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

/*------------3S 锂电池，满电 12.6 V，标称 11.1 V------------*/
#define BATTERY_CELLS             3U
#define BATTERY_LOW_CELL_V        3.50f // 低压报警，3S 共 10.5 V
#define BATTERY_CRITICAL_CELL_V   3.30f // 欠压停机，3S 共 9.9 V
#define BATTERY_HYSTERESIS_CELL_V 0.10f // 回差：比阈值高出 0.1 V/节才算恢复，3S 共 0.3 V
#define BATTERY_FILTER_TAU_S      1.0f  // 电压低通滤波时间常数，压掉电机启停时的瞬间压降
#define BATTERY_CONFIRM_S         2.0f  // 滤波后的电压持续越过阈值这么久才切换状态

typedef enum
{
    BATTERY_UNKNOWN = 0, // 没有有效读数，INA226 未就绪或读取失败
    BATTERY_OK,
    BATTERY_LOW,         // 低压报警，只提示，不影响运动
    BATTERY_CRITICAL     // 欠压停机，必须停车
} BatteryLevel_t;

typedef struct
{
    float filtered_v;       // 滤波后的整包电压
    BatteryLevel_t level;   // 当前状态
    BatteryLevel_t pending; // 正在确认的新状态
    float pending_s;        // pending 已经持续的时间
} BatteryMonitor_t;

/**
 * @brief 清空状态，level 为 BATTERY_UNKNOWN
 */
void BatteryMonitor_Init(BatteryMonitor_t *monitor);

/**
 * @brief 输入一次电压读数，更新电池状态
 * @param bus_v INA226 测得的整包电压，单位 V
 * @param valid 读数是否有效；无效时状态回到 BATTERY_UNKNOWN，下次有效读数重新开始滤波
 * @param dt_s 距上次调用的时间，单位 s
 * @return 更新后的状态；参数非法时返回 BATTERY_UNKNOWN，monitor 不变
 * @note 第一次有效读数直接作为初值并立即判定状态，之后每次切换都要持续 BATTERY_CONFIRM_S
 * @note 传感器故障不当作欠压，不让车停下
 */
BatteryLevel_t BatteryMonitor_Update(BatteryMonitor_t *monitor, float bus_v,
                                     bool valid, float dt_s);

/**
 * @brief 按单节电压估算剩余电量
 * @param pack_v 整包电压，单位 V
 * @return 0～100；非有限值返回 0
 * @note 按静置电压曲线分段插值，行驶时电压被拉低，显示值会偏小
 */
uint8_t BatteryMonitor_Percent(float pack_v);

#endif //XIAOSAI_BATTERY_MONITOR_H
