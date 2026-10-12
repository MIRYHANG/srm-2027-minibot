//
// Created by YZH on 2026/10/12.
//

#ifndef XIAOSAI_NRF_HH_INPUT_H
#define XIAOSAI_NRF_HH_INPUT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hh_logic.h"

/*---------------- 按键去抖 ----------------*/
#define HH_DEBOUNCE_SAMPLE      4U // 连续 4 次采样，每 5 ms 采样一次时约 20 ms

#define HH_BTN_BIT(btn)      ((uint16_t)(1U << (btn)))
#define HH_BTN_VALID_MASK    ((uint16_t)((1U << HH_BTN_COUNT) - 1U))
#define HH_BTN_IMMEDIATE_MASK HH_BTN_BIT(HH_BTN_ESTOP) // 按下立即生效、松开仍要去抖的按键


typedef enum {
    HH_BTN_START = 0,  // 使能键
    HH_BTN_ESTOP,      // 失能键
    HH_BTN_GEAR,       // 摇杆按下，换挡
    HH_BTN_TURN_CCW,   // 逆时针自转键
    HH_BTN_TURN_CW,    // 顺时针自转键
    HH_BTN_ARM_DIR,    // 机械臂方向键
    HH_BTN_ARM_STOP,   // 机械臂停止键
    HH_BTN_J1,         // J1～J5、夹爪必须连续，HhInput_Build 按顺序映射到 joint[]
    HH_BTN_J2,
    HH_BTN_J3,
    HH_BTN_J4,
    HH_BTN_J5,
    HH_BTN_GRIPPER,
    HH_BTN_COUNT
} HhButton_t;

/* 每个按键的去抖状态 */
typedef enum {
    HH_KEY_RELEASED = 0,   // 松开
    HH_KEY_PRESS_CHECK,    // 读到按下，确认中，对外仍算松开
    HH_KEY_PRESSED,        // 按下
    HH_KEY_RELEASE_CHECK   // 读到松开，确认中，对外仍算按下
} HhKeyState_t;

typedef struct
{
    uint16_t stable;                   // 去抖后的按键位图，由各按键状态汇总而来
    HhKeyState_t state[HH_BTN_COUNT];  // 各按键当前所处的状态
    uint8_t count[HH_BTN_COUNT];       // 确认中已经连续读到的次数
} HhDebounce_t;

/**
 * @brief 用第一次采样初始化，上电时就按着的键直接视为按着
 * @param initial 第一次读到的原始位图
 * @note 不从“全部松开”开始，否则上电时按着的使能键会在去抖完成后被当成一次按下
 */
void HhDebounce_Init(HhDebounce_t *debounce, uint16_t initial);

/**
 * @brief 输入一次原始采样，返回去抖后的按键位图
 * @param raw 本次读到的原始位图，HH_BTN_VALID_MASK 以外的位被忽略
 * @return 去抖后的位图；debounce 为 NULL 时返回失能键按下，让手柄失能
 * @note 每个按键是一个四状态状态机：松开 → 确认按下中 → 按下 → 确认松开中 → 松开
 *       确认中要连续 HH_DEBOUNCE_SAMPLE 次读到同一个值才完成转移，中途读到相反的值就退回
 *       失能键例外：松开时读到按下直接进入按下，不经过确认
 */
uint16_t HhDebounce_Update(HhDebounce_t *debounce, uint16_t raw);

/*---------------- 摇杆换算 ----------------*/
#define HH_ADC_MAX                  4095U // 12 位 ADC 最大值
#define HH_STICK_CENTER             2048U // 理想中点，标定失败时使用
#define HH_STICK_CENTER_TOL         300U  // 标定结果偏离理想中点超过它，视为上电时摇杆被推着
#define HH_STICK_DEADZONE           80U   // 中点附近约 2% 的范围输出 0，本质死区
#define HH_STICK_EDGE               60U   // 两端留出的余量，摇杆推不到 ADC 极限也能输出满量程

typedef struct
{
    uint16_t center; // 松手时的 ADC 值
    bool invert;     // 摇杆装反时为 true，输出取反
} HhAxis_t;

/**
 * @brief 初始化一个轴，中点先取理想值
 * @param invert 推向正方向时 ADC 变小则传 true
 */
void HhAxis_Init(HhAxis_t *axis, bool invert);

/**
 * @brief 用上电时松手状态下的若干次采样标定中点
 * @param samples ADC 采样值
 * @param count 采样个数，至少 1 个
 * @return 平均值在理想中点附近则采用并返回 true；参数无效或偏离太多返回 false，中点保持不变
 */
bool HhAxis_Calibrate(HhAxis_t *axis, const uint16_t *samples, size_t count);

/**
 * @brief 把 ADC 原始值换算成 -HH_AXIS_MAX～HH_AXIS_MAX
 * @return 死区内为 0；推到底为 ±HH_AXIS_MAX；axis 为 NULL 时返回 0
 * @note 中点两侧分别换算，中点不在正中间时两边都能推满
 */
int16_t HhAxis_Normalize(const HhAxis_t *axis, uint16_t raw);

/*---------------- 组装输入 ----------------*/
/**
 * @brief 把去抖后的按键位图和换算后的摇杆值组装成 hh_logic 需要的输入
 * @param buttons HhDebounce_Update 的返回值
 * @param forward 前后，HhAxis_Normalize 的结果
 * @param left 横移，HhAxis_Normalize 的结果
 * @param out 输出；为 NULL 时不做任何事
 */
void HhInput_Build(uint16_t buttons, int16_t forward, int16_t left, HhInput_t *out);

#endif //XIAOSAI_NRF_HH_INPUT_H
