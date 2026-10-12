//
// Created by YZH on 2026/10/11.
//

#ifndef XIAOSAI_NRF_HH_LOGIC_H
#define XIAOSAI_NRF_HH_LOGIC_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "handheld_packet.h"

#define HH_LED_BLINK_HALF_MS            100U // 机械臂停止时 LED2 快闪，亮 100 ms 灭 100 ms

typedef struct {
    bool start;                  // 使能键
    bool estop;                  // 失能键（急停）
    bool gear;                   // 摇杆按下，换挡
    bool turn_ccw;               // 逆时针自转键
    bool turn_cw;                // 顺时针自转键
    bool arm_dir;                // 机械臂方向键
    bool arm_stop;               // 机械臂停止键
    bool joint[HH_JOINT_COUNT];  // J1～J5、夹爪
    int16_t forward;             // 前后，正数向前，-1000～1000
    int16_t left;                // 横移，正数向左，-1000～1000
} HhInput_t;

typedef struct
{
    bool enabled;      // 使能锁存：按使能键置位，按失能键清除
    bool arm_stopped;  // 机械臂停止锁存：按停止键置位，按使能键清除
    bool reverse;      // 机械臂方向：false 正向（角度增大），true 反向
    bool last_start;   // 上一次的使能键，用于检测按下沿
    bool last_arm_stop;
    bool dir_held;     // 方向键正按着
    bool dir_combo;    // 这次按住方向键期间按过关节键，松开时不切换方向
    uint8_t seq;       // 下一包的序号
} HhLogic_t;

/**
 * @brief 上电初始化：失能、正向、机械臂未停止
 * @note 按键历史初始化为“按下”，上电时就按着的键必须先松开再按才算一次
 */
void HhLogic_Init(HhLogic_t *logic);

/**
 * @brief 输入一次去抖后的按键，更新锁存状态并生成控制包
 * @param input 本周期的按键和摇杆
 * @param packet 输出控制包；失败时清零
 * @return 成功返回 true；参数为空或摇杆越界返回 false，状态不变
 * @note 每发一包调用一次，按下沿在这里检测
 */
bool HhLogic_Step(HhLogic_t *logic, const HhInput_t *input, HhPacket_t *packet);

/**
 * @brief LED_STATUS2 是否点亮
 * @return 机械臂停止时快闪；否则反向亮、正向灭
 */
bool HhLogic_DirLedOn(const HhLogic_t *logic, uint32_t now_ms);

#endif //XIAOSAI_NRF_HH_LOGIC_H
