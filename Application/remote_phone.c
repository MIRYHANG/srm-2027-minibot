//
// Created by YZH on 2026/9/27.
//

#include "remote_phone.h"
#include <stddef.h>

#define PHONE_ENABLE_MASK 0x01  // S1：允许运动 0000 0001
#define PHONE_STOP_MASK   0x02  // S2：请求停机 0000 0010

static bool PhoneAxisValid(int16_t value)
{
    return value >= -512 && value <= 511;
}

static float PhoneNormalizeAxis(int16_t value)
{
    // 原协议负端是 -512，正端是 511；分别除以对应端点。
    return value < 0 ? (float)value / 512.0f : (float)value / 511.0f;
}

bool RemotePhone_Convert(const srm_control_state_t *raw,
                         RemoteCommand_t *command)
{
    if (command == NULL)
    {
        return false;
    }

    // 转换失败时，输出先保持为安全的全零命令。
    *command = (RemoteCommand_t){0};

    if (raw == NULL ||
        !PhoneAxisValid(raw->left_x) ||
        !PhoneAxisValid(raw->left_y) ||
        !PhoneAxisValid(raw->right_x) ||
        !PhoneAxisValid(raw->right_y))
    {
        return false;
    }

    command->forward = PhoneNormalizeAxis(raw->left_y);
    command->left = PhoneNormalizeAxis(raw->left_x);
    command->turn = PhoneNormalizeAxis(raw->right_x);

    command->enabled = (raw->switches & PHONE_ENABLE_MASK) != 0U;
    command->stop_requested = (raw->switches & PHONE_STOP_MASK) != 0U;

    return true;
}

bool RemotePhone_ProcessByte(srm_parser_t *parser,
                             RemoteInput_t *input,
                             uint8_t byte,
                             uint32_t now_ms)
{
    // 检查解析器和遥控状态是否存在
    if (parser == NULL || input == NULL)
    {
        return false;
    }

    srm_frame_t frame;
    srm_control_state_t raw;
    RemoteCommand_t command;

    // 送入一个字节；没收到完整有效帧，就先结束本次处理
    if (srm_parser_push(parser, byte, &frame) != SRM_PARSE_FRAME)
    {
        return false;
    }

    // 将完整帧解码成手机摇杆、按钮和开关数据
    if (!srm_decode_control(&frame, &raw))
    {
        return false;
    }

    // 将手机数据转换为统一遥控命令
    if (!RemotePhone_Convert(&raw, &command))
    {
        return false;
    }

    // 保存手机命令，并记录这帧有效数据的接收时间
    return RemoteInput_Update(input, REMOTE_SOURCE_PHONE, &command, now_ms);
}