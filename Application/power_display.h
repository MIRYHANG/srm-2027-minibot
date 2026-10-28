//
// Created by YZH on 2026/10/5.
//

#ifndef XIAOSAI_POWER_DISPLAY_H
#define XIAOSAI_POWER_DISPLAY_H

#include <stdbool.h>

#include "battery_monitor.h"
#include "ina226.h"

#define POWER_DISPLAY_LINES 8U  // 对应 OLED 的 8 页
#define POWER_DISPLAY_COLS  21U // 每行最多 21 个字符

typedef char PowerDisplayLine_t[POWER_DISPLAY_COLS + 1U];

typedef enum
{
    RESET_CAUSE_UNKNOWN = 0,
    RESET_CAUSE_POWER,     // 上电或欠压复位
    RESET_CAUSE_PIN,       // 复位按键或调试器复位
    RESET_CAUSE_SOFTWARE,  // 软件复位
    RESET_CAUSE_IWDG,      // 独立看门狗复位
    RESET_CAUSE_WWDG,      // 窗口看门狗复位
    RESET_CAUSE_LOW_POWER, // 低功耗模式非法进入
} ResetCause_t;

/**
 * @brief 生成 OLED 上 8 行文字：标题、电压、电流、功率、传感器状态、复位原因
 * @param reading 最近一次读数，可以为 NULL
 * @param valid 读数是否有效；无效时数值显示为横线
 * @param lines 输出，每行都以 '\0' 结尾且不超过 21 个字符
 * @note 不使用 printf，newlib-nano 默认不支持浮点格式化
 */
void PowerDisplay_Format(const Ina226Reading_t *reading, bool valid,
                         ResetCause_t cause,
                         PowerDisplayLine_t lines[POWER_DISPLAY_LINES]);

/**
 * @brief 生成电池状态行，例如 "BAT  75% OK"、"BAT  12% LOW"
 * @param level 电池状态；BATTERY_UNKNOWN 时电量显示为横线
 * @param percent 估算电量，0～100
 * @param line 输出，以 '\0' 结尾且不超过 21 个字符
 */
void PowerDisplay_FormatBattery(BatteryLevel_t level, uint8_t percent,
                                PowerDisplayLine_t line);

// 遥控链路诊断，ProtocolTask 每周期更新一份
typedef struct
{
    uint32_t rx_bytes;  // 串口收到的字节数
    uint32_t rx_frames; // 校验通过的手机帧数
    bool online;        // 当前来源在超时时间内有有效帧
    bool s1;            // 最近一帧的 S1（使能）
    bool s2;            // 最近一帧的 S2（停机）
    uint8_t safety;     // SafetyInput 有效位
    bool enabled;       // RobotCmd 是否已使能
    uint8_t arm_joint;  // 手机当前选中的关节，1～6，6 为夹爪
    float arm_jog;      // 发给机械臂的该关节 jog，-1～1
    bool arm_enabled;   // 发给机械臂的命令是否使能
} PowerDisplayRemote_t;

/**
 * @brief 生成遥控诊断行，例如 "R123 F045 S10 X10 E0"
 * @note R/F 为字节数和帧数的后三位；S 为 S1、S2，离线时显示 "S--"；
 *       X 为安全输入的十六进制位，E 为是否已使能
 */
void PowerDisplay_FormatRemote(const PowerDisplayRemote_t *remote,
                               PowerDisplayLine_t line);

/**
 * @brief 生成机械臂诊断行，例如 "J1 G+100 P1500 A1"
 * @param pulse_us 选中关节当前输出的脉宽
 * @note J 为选中关节，G 为 jog 百分比，P 为脉宽，A 为机械臂命令是否使能
 */
void PowerDisplay_FormatArm(const PowerDisplayRemote_t *remote, uint16_t pulse_us,
                            PowerDisplayLine_t line);

#endif //XIAOSAI_POWER_DISPLAY_H
