//
// Created by YZH on 2026/9/26
//

#ifndef XIAOSAI_REMOTE_INPUT_H
#define XIAOSAI_REMOTE_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_pose.h"

typedef enum
{
    REMOTE_SOURCE_NONE = 0,     // 没有选择遥控器
    REMOTE_SOURCE_PHONE,        // 手机，值为 1
    REMOTE_SOURCE_HANDHELD      // 自制手柄，值为 2
} RemoteSource_t;

typedef enum
{
    ARM_PRESET_NONE = 0,   // 不请求预设位姿
    ARM_PRESET_GRAB_READY, // 抓取准备位姿
    ARM_PRESET_STOW        // 收纳位姿
} ArmPreset_t;

// 手机和手柄共用的机械臂命令，所有字段都是当前帧的电平状态
typedef struct
{
    float jog[ARM_JOINT_COUNT]; // 各关节运动方向和速度比例，范围 -1～1，零表示保持
    ArmPreset_t preset;         // 本帧请求的预设位姿，NONE 表示不请求
    bool gripper_close;         // 请求夹紧
    bool gripper_open;          // 请求松开，同时请求开和关时视为无请求
} ArmRemoteCommand_t;

typedef struct
{
    float forward;         // 前后，范围 -1.0f ~ 1.0f
    float left;            // 左右，范围 -1.0f ~ 1.0f
    float turn;            // 旋转，范围 -1.0f ~ 1.0f
    bool enabled;          // 是否允许输出运动命令
    bool stop_requested;   // 软件停机请求
    ArmRemoteCommand_t arm; // 机械臂命令，全部为零时表示保持当前位置
} RemoteCommand_t;

typedef struct
{
    RemoteCommand_t command;
    uint32_t last_valid_ms; // 最近一次收到有效遥控帧的时间
    bool has_valid_frame;
    bool armed;             // 已收到 enabled=false 的有效帧；状态清零后为 false，超时不清除
} RemoteState_t;

typedef struct
{
    RemoteState_t phone;            // 手机最近发来的命令和接收时间
    RemoteState_t handheld;         // 自制手柄最近发来的命令和接收时间
    RemoteSource_t selected;        // 当前选择手机、手柄，还是都不选
} RemoteInput_t;

void RemoteInput_Init(RemoteInput_t *input);
void RemoteInput_Select(RemoteInput_t *input, RemoteSource_t source);
bool RemoteInput_Update(RemoteInput_t *input, RemoteSource_t source,
                        const RemoteCommand_t *command, uint32_t now_ms);
RemoteCommand_t RemoteInput_GetSafe(const RemoteInput_t *input,
                                    uint32_t now_ms, uint32_t timeout_ms);

#endif
