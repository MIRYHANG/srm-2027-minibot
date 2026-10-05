//
// Created by YZH on 2026/9/30.
//

#ifndef XIAOSAI_ARM_CONTROL_H
#define XIAOSAI_ARM_CONTROL_H

#include "arm_pose.h"
#include "remote_input.h"

typedef struct
{
    ArmPose_t target;
    bool has_target;
} ArmControl_t;

/**
 * @brief 初始化机械臂控制状态
 */
void ArmControl_Init(ArmControl_t *control);

/**
 * @brief 检查并保存新的目标姿态
 * @return 保存成功返回 true，否则返回 false
 * @note 这里只保存目标，不驱动舵机
 */
bool ArmControl_SetTarget(ArmControl_t *control, const ArmPose_t *pose);

/**
 * @brief 读取已保存的目标姿态
 * @param out_pose 用于接收目标姿态
 * @return 有目标返回 true，没有目标返回 false
 */
bool ArmControl_GetTarget(const ArmControl_t *control, ArmPose_t *out_pose);

/**
 * @brief 清除目标姿态
 * @note 只清除缓存，不会停止正在输出的 PWM
 */
void ArmControl_Clear(ArmControl_t *control);

/**
 * @brief 读取预设位姿
 * @param preset 预设编号
 * @param out 用于接收预设位姿，包含夹爪的值
 * @return 预设有效返回 true；NONE、非法值或 out 为空返回 false，且不修改 out
 */
bool ArmControl_GetPreset(ArmPreset_t preset, ArmPose_t *out);

/**
 * @brief 根据遥控命令更新目标位姿
 * @param control 控制状态
 * @param current 当前位姿，jog 以它为基准，没有目标时用它初始化目标
 * @param cmd 机械臂遥控命令，各字段为电平语义，全零表示保持
 * @param dt_s 本次更新的时间间隔，单位为秒
 * @return 成功返回 true；输入非法或结果越界返回 false，且不修改 control
 * @note 预设只改 J1～J5，夹爪只由 gripper_close / gripper_open 控制
 * @note 这里只更新目标，不驱动舵机
 */
bool ArmControl_Update(ArmControl_t *control, const ArmPose_t *current,
                       const ArmRemoteCommand_t *cmd, float dt_s);
#endif //XIAOSAI_ARM_CONTROL_H
