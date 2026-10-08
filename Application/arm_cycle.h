//
// Created by YZH on 2026/10/5.
//

#ifndef XIAOSAI_ARM_CYCLE_H
#define XIAOSAI_ARM_CYCLE_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_control.h"
#include "arm_pose.h"
#include "robot_def.h"

// 单周期允许的最大时间步，超过时按此值计算，防止任务被延迟后一步跳得太远
#define ARM_CYCLE_MAX_DT_S 0.06f

typedef struct
{
    ArmControl_t control;
    ArmPose_t current;                  // 最近一次输出的位姿；舵机没有反馈，用输出值代替实际位置
    uint16_t pulse_us[ARM_JOINT_COUNT]; // current 对应的脉宽，按关节顺序排列
    bool initialized;
} ArmCycle_t;

/**
 * @brief 以 STOW 位姿初始化，current、target 和脉宽都对应 STOW
 * @return 成功返回 true；失败返回 false，且不修改 cycle
 * @note 启动舵机 PWM 之前调用，保证舵机收到的第一个脉宽就是 STOW
 */
bool ArmCycle_Init(ArmCycle_t *cycle);

/**
 * @brief 执行一个控制周期：更新目标 → 限速逼近 → 换算脉宽
 * @param cycle 已初始化的状态
 * @param cmd 本周期的机械臂命令
 * @param dt_s 距上次调用的时间，单位为秒，超过 ARM_CYCLE_MAX_DT_S 时按上限计算
 * @return 成功返回 true；任一步失败返回 false，且不修改 cycle
 * @note 未使能或请求停机时，目标拉回当前位置，机械臂当场停住但不卸力
 */
bool ArmCycle_Step(ArmCycle_t *cycle, const ArmCmd_t *cmd, float dt_s);

#endif //XIAOSAI_ARM_CYCLE_H