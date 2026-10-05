//
// Created by YZH on 2026/10/5.
//

#ifndef XIAOSAI_POWER_DISPLAY_H
#define XIAOSAI_POWER_DISPLAY_H

#include <stdbool.h>

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

#endif //XIAOSAI_POWER_DISPLAY_H
