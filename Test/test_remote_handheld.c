//
// Created by YZH on 2026/10/11.
//
#include <assert.h>
#include <stdio.h>

#include "remote_handheld.h"

static RemoteCommand_t NonzeroCommand(void)
{
    RemoteCommand_t command = {
        .forward = 1.0f,
        .left = 1.0f,
        .turn = 1.0f,
        .enabled = true,
        .stop_requested = true,
        .arm_stop = true,
        .gear_button = true,
        .arm = {.preset = ARM_PRESET_STOW}
    };
    for (int idx = 0; idx < ARM_JOINT_COUNT; ++idx)
    {
        command.arm.jog[idx] = 1.0f;
    }
    return command;
}

static void ExpectZero(const RemoteCommand_t *command)
{
    assert(command->forward == 0.0f);
    assert(command->left == 0.0f);
    assert(command->turn == 0.0f);
    assert(!command->enabled);
    assert(!command->stop_requested);
    assert(!command->arm_stop);
    assert(!command->gear_button);
    assert(command->arm.preset == ARM_PRESET_NONE);
    for (int idx = 0; idx < ARM_JOINT_COUNT; ++idx)
    {
        assert(command->arm.jog[idx] == 0.0f);
    }
}

static size_t EncodePacket(const HhPacket_t *packet, uint8_t *out)
{
    size_t length = HhPacket_Encode(packet, out, HH_PACKET_LEN);
    assert(length == HH_PACKET_LEN);
    return length;
}

static void TestAxesScale(void)
{
    HhPacket_t packet = {.forward = 1000, .left = -500};
    RemoteCommand_t command = NonzeroCommand();

    assert(RemoteHandheld_Convert(&packet, &command));
    assert(command.forward == 1.0f);
    assert(command.left == -0.5f);
    assert(command.turn == 0.0f);

    packet.forward = -1000;
    packet.left = 1000;
    assert(RemoteHandheld_Convert(&packet, &command));
    assert(command.forward == -1.0f);
    assert(command.left == 1.0f);
}

static void TestTurnButtons(void)
{
    HhPacket_t packet = {.flags = HH_FLAG_TURN_CCW};
    RemoteCommand_t command;

    assert(RemoteHandheld_Convert(&packet, &command));
    assert(command.turn == 1.0f); // 逆时针为正

    packet.flags = HH_FLAG_TURN_CW;
    assert(RemoteHandheld_Convert(&packet, &command));
    assert(command.turn == -1.0f);

    packet.flags = HH_FLAG_TURN_CCW | HH_FLAG_TURN_CW;
    assert(RemoteHandheld_Convert(&packet, &command));
    assert(command.turn == 0.0f); // 两个同时按住时不转
}

static void TestFlagsMapping(void)
{
    HhPacket_t packet = {.flags = 0U};
    RemoteCommand_t command = NonzeroCommand();

    assert(RemoteHandheld_Convert(&packet, &command));
    ExpectZero(&command);

    packet.flags = HH_FLAG_ENABLED;
    assert(RemoteHandheld_Convert(&packet, &command));
    assert(command.enabled && !command.stop_requested && !command.arm_stop && !command.gear_button);

    packet.flags = HH_FLAG_ESTOP;
    assert(RemoteHandheld_Convert(&packet, &command));
    assert(!command.enabled && command.stop_requested);

    packet.flags = HH_FLAG_ARM_STOP;
    assert(RemoteHandheld_Convert(&packet, &command));
    assert(command.arm_stop);

    packet.flags = HH_FLAG_GEAR;
    assert(RemoteHandheld_Convert(&packet, &command));
    assert(command.gear_button);
}

static void TestArmMapping(void)
{
    HhPacket_t packet = {.jog = {100, -100, 50, 0, -1, 100}};
    RemoteCommand_t command;

    assert(RemoteHandheld_Convert(&packet, &command));
    assert(command.arm.jog[ARM_J1] == 1.0f);
    assert(command.arm.jog[ARM_J2] == -1.0f);
    assert(command.arm.jog[ARM_J3] == 0.5f);
    assert(command.arm.jog[ARM_J4] == 0.0f);
    assert(command.arm.jog[ARM_J5] == -0.01f);
    assert(command.arm.jog[ARM_GRIPPER] == 1.0f);
    assert(command.arm.preset == ARM_PRESET_NONE);

    packet.preset = HH_PRESET_READY;
    assert(RemoteHandheld_Convert(&packet, &command));
    assert(command.arm.preset == ARM_PRESET_GRAB_READY);

    packet.preset = HH_PRESET_STOW;
    assert(RemoteHandheld_Convert(&packet, &command));
    assert(command.arm.preset == ARM_PRESET_STOW);
}

/* Convert 只接受已校验的包，但仍要防住越界值，避免进入底盘和机械臂 */
static void TestConvertRejectsInvalid(void)
{
    RemoteCommand_t command = NonzeroCommand();
    HhPacket_t packet = {.forward = 1001};

    assert(!RemoteHandheld_Convert(&packet, &command));
    ExpectZero(&command);

    command = NonzeroCommand();
    packet = (HhPacket_t){.preset = 3U};
    assert(!RemoteHandheld_Convert(&packet, &command));
    ExpectZero(&command);

    command = NonzeroCommand();
    packet = (HhPacket_t){.jog = {0, 0, 0, 0, 0, -101}};
    assert(!RemoteHandheld_Convert(&packet, &command));
    ExpectZero(&command);

    assert(!RemoteHandheld_Convert(NULL, &command));
    assert(!RemoteHandheld_Convert(&packet, NULL));
}

/* 和手机一样：先收到一帧未使能才上锁，之后的使能帧才能输出 */
static void TestProcessPayloadUpdatesInput(void)
{
    RemoteInput_t input;
    RemoteInput_Init(&input);
    RemoteInput_Select(&input, REMOTE_SOURCE_HANDHELD);

    uint8_t data[HH_PACKET_LEN];
    HhPacket_t packet = {.forward = 1000, .flags = HH_FLAG_ENABLED};

    // 上电后直接收到使能帧：有效但尚未上锁，安全命令为零
    EncodePacket(&packet, data);
    assert(RemoteHandheld_ProcessPayload(&input, data, sizeof(data), 100U));
    assert(input.handheld.has_valid_frame);
    assert(input.handheld.last_valid_ms == 100U);
    assert(!input.handheld.armed);
    RemoteCommand_t safe = RemoteInput_GetSafe(&input, 101U, 100U);
    ExpectZero(&safe);

    packet.flags = 0U;
    EncodePacket(&packet, data);
    assert(RemoteHandheld_ProcessPayload(&input, data, sizeof(data), 110U));
    assert(input.handheld.armed);

    packet.flags = HH_FLAG_ENABLED;
    EncodePacket(&packet, data);
    assert(RemoteHandheld_ProcessPayload(&input, data, sizeof(data), 120U));
    safe = RemoteInput_GetSafe(&input, 121U, 100U);
    assert(safe.enabled);
    assert(safe.forward == 1.0f);

    // 手机那一路不受影响
    assert(!input.phone.has_valid_frame);
}

static void TestProcessPayloadRejectsBadData(void)
{
    RemoteInput_t input;
    RemoteInput_Init(&input);
    RemoteInput_Select(&input, REMOTE_SOURCE_HANDHELD);

    uint8_t data[HH_PACKET_LEN];
    HhPacket_t packet = {.flags = 0U};
    EncodePacket(&packet, data);
    assert(RemoteHandheld_ProcessPayload(&input, data, sizeof(data), 50U));

    // 坏包、长度不对、旧版的单字节计数包都不能刷新接收时间
    data[3] ^= 0x10U;
    assert(!RemoteHandheld_ProcessPayload(&input, data, sizeof(data), 60U));
    uint8_t counter[1] = {7U};
    assert(!RemoteHandheld_ProcessPayload(&input, counter, sizeof(counter), 70U));
    assert(!RemoteHandheld_ProcessPayload(&input, NULL, HH_PACKET_LEN, 80U));
    assert(!RemoteHandheld_ProcessPayload(NULL, data, sizeof(data), 90U));
    assert(input.handheld.last_valid_ms == 50U);
}

int main(void)
{
    TestAxesScale();
    TestTurnButtons();
    TestFlagsMapping();
    TestArmMapping();
    TestConvertRejectsInvalid();
    TestProcessPayloadUpdatesInput();
    TestProcessPayloadRejectsBadData();

    puts("Remote handheld tests passed");
    return 0;
}
