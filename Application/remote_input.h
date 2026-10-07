//
// Created by YZH on 2026/9/26
//

#ifndef XIAOSAI_REMOTE_INPUT_H
#define XIAOSAI_REMOTE_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_def.h"

typedef enum
{
    REMOTE_SOURCE_NONE = 0,     // 没有选择遥控器
    REMOTE_SOURCE_PHONE,        // 手机，值为 1
    REMOTE_SOURCE_HANDHELD      // 自制手柄，值为 2
} RemoteSource_t;

typedef struct
{
    float forward;         // 前后，范围 -1.0f ~ 1.0f
    float left;            // 左右，范围 -1.0f ~ 1.0f
    float turn;            // 旋转，范围 -1.0f ~ 1.0f
    bool enabled;          // 是否允许输出运动命令
    bool stop_requested;   // 软件停机请求
    ArmMotionCmd_t arm;    // 机械臂命令，全部为零时表示保持当前位置
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
bool RemoteInput_OperatorDisabled(const RemoteInput_t *input,
                                  uint32_t now_ms, uint32_t timeout_ms);

#endif
