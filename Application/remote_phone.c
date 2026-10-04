//
// Created by YZH on 2026/9/27
//

#include "remote_phone.h"
#include <stddef.h>

#define PHONE_ENABLE_MASK 0x01  // S1：允许运动 0000 0001
#define PHONE_STOP_MASK   0x02  // S2：请求停机 0000 0010

#define PHONE_BUTTON_A 0x01U
#define PHONE_BUTTON_B 0x02U
#define PHONE_BUTTON_X 0x04U
#define PHONE_BUTTON_Y 0x08U

#define PHONE_DPAD_UP    1U
#define PHONE_DPAD_DOWN  2U
#define PHONE_DPAD_LEFT  3U
#define PHONE_DPAD_RIGHT 4U

static bool PhoneAxisValid(int16_t value)
{
    return value >= -512 && value <= 511;
}

static float PhoneNormalizeAxis(int16_t value)
{
    // 原协议负端是 -512，正端是 511；分别除以对应端点
    return value < 0 ? (float)value / 512.0f : (float)value / 511.0f;
}

void RemotePhoneArm_Init(RemotePhoneArm_t *arm)
{
    if (arm == NULL)
    {
        return;
    }

    arm->selected = ARM_J1;
    arm->last_dpad = 0U;
}

int RemotePhoneArm_GetSelected(const RemotePhoneArm_t *arm)
{
    if (arm == NULL ||
        arm->selected < ARM_J1 ||
        arm->selected >= ARM_JOINT_COUNT)
    {
        return -1;
    }

    return arm->selected;
}

bool RemotePhone_Convert(const srm_control_state_t *raw,
                         RemotePhoneArm_t *arm,
                         RemoteCommand_t *command)
{
    if (command == NULL)
    {
        return false;
    }

    // 转换失败时，输出先保持为安全的全零命令
    *command = (RemoteCommand_t){0};

    if (raw == NULL ||
        RemotePhoneArm_GetSelected(arm) < 0 ||
        !PhoneAxisValid(raw->left_x) ||
        !PhoneAxisValid(raw->left_y) ||
        !PhoneAxisValid(raw->right_x) ||
        !PhoneAxisValid(raw->right_y) ||
        raw->dpad > PHONE_DPAD_RIGHT ||
        (raw->buttons & 0xF0U) != 0U)
    {
        return false;
    }

    int selected = arm->selected;

    if (raw->dpad == PHONE_DPAD_UP && arm->last_dpad != PHONE_DPAD_UP)
    {
        ++selected;
        if (selected >= ARM_JOINT_COUNT)
        {
            selected = ARM_J1;
        }
    }
    else if (raw->dpad == PHONE_DPAD_DOWN && arm->last_dpad != PHONE_DPAD_DOWN)
    {
        --selected;
        if (selected < ARM_J1)
        {
            selected = ARM_GRIPPER;
        }
    }

    RemoteCommand_t next = {0};
    next.forward = PhoneNormalizeAxis(raw->left_y);
    next.left = PhoneNormalizeAxis(raw->left_x);

    if (raw->dpad == PHONE_DPAD_LEFT)
    {
        next.turn = 1.0f;  // 逆时针为正
    }
    else if (raw->dpad == PHONE_DPAD_RIGHT)
    {
        next.turn = -1.0f;
    }

    // right_x 不参与控制，但前面仍检查了它的合法范围
    next.arm.jog[selected] = PhoneNormalizeAxis(raw->right_y);

    bool a = (raw->buttons & PHONE_BUTTON_A) != 0U;
    bool b = (raw->buttons & PHONE_BUTTON_B) != 0U;

    if (a && !b)
    {
        next.arm.preset = ARM_PRESET_GRAB_READY;
    }
    else if (b && !a)
    {
        next.arm.preset = ARM_PRESET_STOW;
    }

    next.arm.gripper_close = (raw->buttons & PHONE_BUTTON_X) != 0U;
    next.arm.gripper_open = (raw->buttons & PHONE_BUTTON_Y) != 0U;

    next.enabled = (raw->switches & PHONE_ENABLE_MASK) != 0U;
    next.stop_requested = (raw->switches & PHONE_STOP_MASK) != 0U;

    // 所有校验和转换成功后，才提交输出与按键历史
    *command = next;
    arm->selected = selected;
    arm->last_dpad = raw->dpad;
    return true;
}

bool RemotePhone_ProcessByte(srm_parser_t *parser,
                             RemoteInput_t *input,
                             RemotePhoneArm_t *arm,
                             uint8_t byte,
                             uint32_t now_ms)
{
    // 检查解析器和遥控状态是否存在
    if (parser == NULL || input == NULL || arm == NULL)
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
    if (!RemotePhone_Convert(&raw, arm, &command))
    {
        return false;
    }

    // 保存手机命令，并记录这帧有效数据的接收时间
    return RemoteInput_Update(input, REMOTE_SOURCE_PHONE, &command, now_ms);
}
