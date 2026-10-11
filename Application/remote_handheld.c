//
// Created by YZH on 2026/10/11.
//

#include "remote_handheld.h"

_Static_assert(HH_JOINT_COUNT == ARM_JOINT_COUNT, "手柄关节数与机械臂关节数不一致");

static bool HasFlag(const HhPacket_t *packet, uint8_t flag)
{
    return (packet->flags & flag) != 0U;
}

static bool PacketInRange(const HhPacket_t *packet)
{
    if (packet->forward < -HH_AXIS_MAX || packet->forward > HH_AXIS_MAX ||
        packet->left < -HH_AXIS_MAX || packet->left > HH_AXIS_MAX ||
        packet->preset > HH_PRESET_STOW)
    {
        return false;
    }

    for (int idx = 0; idx < ARM_JOINT_COUNT; ++idx)
    {
        if (packet->jog[idx] < -HH_JOG_MAX || packet->jog[idx] > HH_JOG_MAX)
        {
            return false;
        }
    }

    return true;
}

static float TurnFromButtons(const HhPacket_t *packet)
{
    bool ccw = HasFlag(packet, HH_FLAG_TURN_CCW);
    bool cw = HasFlag(packet, HH_FLAG_TURN_CW);

    if (ccw && !cw)
    {
        return 1.0f;  // 逆时针为正
    }
    if (cw && !ccw)
    {
        return -1.0f;
    }
    return 0.0f;      // 都没按或同时按住
}

static ArmPreset_t PresetFromPacket(uint8_t preset)
{
    switch (preset)
    {
        case HH_PRESET_READY:
            return ARM_PRESET_GRAB_READY;
        case HH_PRESET_STOW:
            return ARM_PRESET_STOW;
        default:
            return ARM_PRESET_NONE;
    }
}

bool RemoteHandheld_Convert(const HhPacket_t *packet, RemoteCommand_t *command)
{
    if (command == NULL)
    {
        return false;
    }

    // 转换失败时，输出保持为安全的全零命令
    *command = (RemoteCommand_t){0};

    if (packet == NULL || !PacketInRange(packet))
    {
        return false;
    }

    RemoteCommand_t next = {0};
    next.forward = (float)packet->forward / (float)HH_AXIS_MAX;
    next.left = (float)packet->left / (float)HH_AXIS_MAX;
    next.turn = TurnFromButtons(packet);
    next.enabled = HasFlag(packet, HH_FLAG_ENABLED);
    next.stop_requested = HasFlag(packet, HH_FLAG_ESTOP);
    next.arm_stop = HasFlag(packet, HH_FLAG_ARM_STOP);
    next.gear_button = HasFlag(packet, HH_FLAG_GEAR);

    for (int idx = 0; idx < ARM_JOINT_COUNT; ++idx)
    {
        next.arm.jog[idx] = (float)packet->jog[idx] / (float)HH_JOG_MAX;
    }
    next.arm.preset = PresetFromPacket(packet->preset);

    *command = next;
    return true;
}

bool RemoteHandheld_ProcessPayload(RemoteInput_t *input, const uint8_t *payload,
                                   size_t length, uint32_t now_ms)
{
    if (input == NULL || payload == NULL)
    {
        return false;
    }

    HhPacket_t packet;
    RemoteCommand_t command;

    if (!HhPacket_Decode(payload, length, &packet))
    {
        return false;
    }

    if (!RemoteHandheld_Convert(&packet, &command))
    {
        return false;
    }

    return RemoteInput_Update(input, REMOTE_SOURCE_HANDHELD, &command, now_ms);
}
