//
// Created by YZH on 2026/9/26.
//

#ifndef XIAOSAI_REMOTE_INPUT_H
#define XIAOSAI_REMOTE_INPUT_H

#include <stdbool.h>
#include <stdint.h>

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
    bool enabled;          // 是否允许底盘运动
    bool stop_requested;   // 软件停机请求
} RemoteCommand_t;

typedef struct
{
    RemoteCommand_t command;
    uint32_t last_valid_ms; // 最近一次收到有效遥控帧的时间
    bool has_valid_frame;
} RemoteState_t;

typedef struct
{
    RemoteState_t phone;
    RemoteState_t handheld;
    RemoteSource_t selected;
} RemoteInput_t;

void RemoteInput_Init(RemoteInput_t *input);
void RemoteInput_Select(RemoteInput_t *input, RemoteSource_t source);
bool RemoteInput_Update(RemoteInput_t *input, RemoteSource_t source,
                        const RemoteCommand_t *command, uint32_t now_ms);
RemoteCommand_t RemoteInput_GetSafe(const RemoteInput_t *input,
                                    uint32_t now_ms, uint32_t timeout_ms);

#endif
