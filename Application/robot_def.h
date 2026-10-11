//
// Created by YZH on 2026/10/7.
//

#ifndef XIAOSAI_ROBOT_DEF_H
#define XIAOSAI_ROBOT_DEF_H

#include <stdbool.h>
#include <stdint.h>
#include "arm_pose.h"

typedef enum
{
    ARM_PRESET_NONE = 0,
    ARM_PRESET_GRAB_READY,
    ARM_PRESET_STOW
} ArmPreset_t;

typedef struct
{
    float jog[ARM_JOINT_COUNT]; // 各关节运动方向和速度比例，范围 -1～1，零表示保持
    ArmPreset_t preset;
} ArmMotionCmd_t;

/*-------robot_cmd 发给机械臂任务的命令--------*/
typedef struct
{
    bool enabled;          // false 时机械臂当场停住，不卸力
    ArmMotionCmd_t motion; // enabled 为 false 时全零
} ArmCmd_t;

/*-------robot_cmd 发给底盘任务的命令--------*/
typedef struct
{
    bool enabled;  // false 时底盘停车
    float forward; // 前后，范围 -1～1
    float left;    // 左右，范围 -1～1
    float turn;    // 旋转，范围 -1～1，逆时针为正
    uint8_t gear;
} ChassisCmd_t;

#endif //XIAOSAI_ROBOT_DEF_H