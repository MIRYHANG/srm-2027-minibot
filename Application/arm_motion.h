//
// Created by YZH on 2026/10/3.
//

#ifndef XIAOSAI_ARM_MOTION_H
#define XIAOSAI_ARM_MOTION_H
#include <stdbool.h>
#include "arm_pose.h"

/**
 * @brief 让舵机从当前值向目标值移动一小步
 * @param current 当前位姿
 * @param target 目标位姿，越界值会裁剪到标定限位
 * @param dt_s 本次更新的时间间隔，单位为秒
 * @param out 输出下一步位姿，允许与 current 指向同一对象
 * @return 全部关节计算成功返回 true，否则返回 false 且不修改 out
 */
bool ArmMotion_Step(const ArmPose_t *current,
                    const ArmPose_t *target,
                    float dt_s,
                    ArmPose_t *out);

#endif //XIAOSAI_ARM_MOTION_H
