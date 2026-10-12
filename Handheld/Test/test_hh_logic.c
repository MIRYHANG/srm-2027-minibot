//
// Created by YZH on 2026/10/12.
//
#include <assert.h>
#include <stdio.h>

#include "hh_logic.h"

#define J1 0
#define J2 1
#define J3 2
#define GRIPPER 5

static HhPacket_t Step(HhLogic_t *logic, HhInput_t input)
{
    HhPacket_t packet;
    assert(HhLogic_Step(logic, &input, &packet));
    return packet;
}

static void ExpectNoMotion(const HhPacket_t *packet)
{
    assert(packet->forward == 0);
    assert(packet->left == 0);
    assert(packet->preset == HH_PRESET_NONE);
    for (int idx = 0; idx < (int)HH_JOINT_COUNT; ++idx)
    {
        assert(packet->jog[idx] == 0);
    }
}

/* 上电后松开所有键，再按一下使能键，得到已使能的状态 */
static HhLogic_t EnabledLogic(void)
{
    HhLogic_t logic;
    HhLogic_Init(&logic);
    (void)Step(&logic, (HhInput_t){0});
    HhPacket_t packet = Step(&logic, (HhInput_t){.start = true});
    assert(packet.flags & HH_FLAG_ENABLED);
    (void)Step(&logic, (HhInput_t){0});
    return logic;
}

static void TestPowerOnDisabled(void)
{
    HhLogic_t logic;
    HhLogic_Init(&logic);

    HhPacket_t packet = Step(&logic, (HhInput_t){.forward = 1000, .joint = {true}});
    assert(packet.flags == 0U);
    ExpectNoMotion(&packet);
    assert(!HhLogic_DirLedOn(&logic, 0U));
}

/* 上电时使能键就按着（卡住或误按），必须松开再按才使能 */
static void TestStartHeldAtPowerOnIgnored(void)
{
    HhLogic_t logic;
    HhLogic_Init(&logic);

    HhPacket_t packet = Step(&logic, (HhInput_t){.start = true});
    assert(!(packet.flags & HH_FLAG_ENABLED));
    packet = Step(&logic, (HhInput_t){.start = true});
    assert(!(packet.flags & HH_FLAG_ENABLED));

    (void)Step(&logic, (HhInput_t){0});
    packet = Step(&logic, (HhInput_t){.start = true});
    assert(packet.flags & HH_FLAG_ENABLED);
}

/* 使能是锁存的：松开使能键后保持使能 */
static void TestEnableLatches(void)
{
    HhLogic_t logic = EnabledLogic();
    HhPacket_t packet = Step(&logic, (HhInput_t){.forward = -1000, .left = 500});

    assert(packet.flags & HH_FLAG_ENABLED);
    assert(packet.forward == -1000);
    assert(packet.left == 500);
}

static void TestEstopDisablesAndBlocksStart(void)
{
    HhLogic_t logic = EnabledLogic();

    HhPacket_t packet = Step(&logic, (HhInput_t){.estop = true, .forward = 1000});
    assert(!(packet.flags & HH_FLAG_ENABLED));
    assert(packet.flags & HH_FLAG_ESTOP);
    ExpectNoMotion(&packet);

    // 失能键按住时，按使能键无效
    packet = Step(&logic, (HhInput_t){.estop = true, .start = true});
    assert(!(packet.flags & HH_FLAG_ENABLED));

    // 松开失能键后仍然失能，使能键也要重新按一次
    packet = Step(&logic, (HhInput_t){.start = true});
    assert(!(packet.flags & HH_FLAG_ENABLED));
    assert(!(packet.flags & HH_FLAG_ESTOP));
    (void)Step(&logic, (HhInput_t){0});
    packet = Step(&logic, (HhInput_t){.start = true});
    assert(packet.flags & HH_FLAG_ENABLED);
}

static void TestTurnAndGearPassThrough(void)
{
    HhLogic_t logic = EnabledLogic();

    HhPacket_t packet = Step(&logic, (HhInput_t){.turn_ccw = true, .gear = true});
    assert(packet.flags & HH_FLAG_TURN_CCW);
    assert(packet.flags & HH_FLAG_GEAR);
    assert(!(packet.flags & HH_FLAG_TURN_CW));

    packet = Step(&logic, (HhInput_t){.turn_cw = true});
    assert(packet.flags & HH_FLAG_TURN_CW);
    assert(!(packet.flags & HH_FLAG_GEAR));
}

/* 失能时不发送任何运动量，自转和换挡也不发 */
static void TestDisabledSendsNoMotionFlags(void)
{
    HhLogic_t logic;
    HhLogic_Init(&logic);
    (void)Step(&logic, (HhInput_t){0});

    HhPacket_t packet = Step(&logic, (HhInput_t){.turn_ccw = true, .gear = true,
                                                 .joint = {true, true}});
    assert(packet.flags == 0U);
    ExpectNoMotion(&packet);
}

static void TestJointJogForwardAndMultiple(void)
{
    HhLogic_t logic = EnabledLogic();

    HhPacket_t packet = Step(&logic, (HhInput_t){.joint = {[J1] = true, [J3] = true, [GRIPPER] = true}});
    assert(packet.jog[J1] == HH_JOG_MAX);
    assert(packet.jog[J2] == 0);
    assert(packet.jog[J3] == HH_JOG_MAX);
    assert(packet.jog[GRIPPER] == HH_JOG_MAX); // 正向夹紧

    packet = Step(&logic, (HhInput_t){0});
    ExpectNoMotion(&packet);                   // 松开立刻停
}

/* 方向键单独短按：松开时切换方向 */
static void TestDirShortPressToggles(void)
{
    HhLogic_t logic = EnabledLogic();

    (void)Step(&logic, (HhInput_t){.arm_dir = true});
    assert(!logic.reverse);                    // 按下时还不切换
    (void)Step(&logic, (HhInput_t){0});
    assert(logic.reverse);
    assert(HhLogic_DirLedOn(&logic, 0U));

    HhPacket_t packet = Step(&logic, (HhInput_t){.joint = {[J2] = true, [GRIPPER] = true}});
    assert(packet.jog[J2] == -HH_JOG_MAX);
    assert(packet.jog[GRIPPER] == -HH_JOG_MAX); // 反向松开

    (void)Step(&logic, (HhInput_t){.arm_dir = true});
    (void)Step(&logic, (HhInput_t){0});
    assert(!logic.reverse);
    assert(!HhLogic_DirLedOn(&logic, 0U));
}

static void TestDirComboPresets(void)
{
    HhLogic_t logic = EnabledLogic();

    // 方向键 + J1：STOW；按住期间关节不转，松开后方向不变
    (void)Step(&logic, (HhInput_t){.arm_dir = true});
    HhPacket_t packet = Step(&logic, (HhInput_t){.arm_dir = true, .joint = {[J1] = true}});
    assert(packet.preset == HH_PRESET_STOW);
    assert(packet.jog[J1] == 0);
    (void)Step(&logic, (HhInput_t){0});
    assert(!logic.reverse);

    // 方向键 + J2：READY
    (void)Step(&logic, (HhInput_t){.arm_dir = true});
    packet = Step(&logic, (HhInput_t){.arm_dir = true, .joint = {[J2] = true}});
    assert(packet.preset == HH_PRESET_READY);
    (void)Step(&logic, (HhInput_t){.arm_dir = true});   // 先松开 J2 再松方向键
    (void)Step(&logic, (HhInput_t){0});
    assert(!logic.reverse);

    // J1、J2 同时按：含义不明确，不发预设
    packet = Step(&logic, (HhInput_t){.arm_dir = true, .joint = {[J1] = true, [J2] = true}});
    assert(packet.preset == HH_PRESET_NONE);
    (void)Step(&logic, (HhInput_t){0});
    assert(!logic.reverse);
}

/* 方向键 + 其他关节键：既不转也不切换方向 */
static void TestDirWithOtherJointDoesNothing(void)
{
    HhLogic_t logic = EnabledLogic();

    (void)Step(&logic, (HhInput_t){.arm_dir = true});
    HhPacket_t packet = Step(&logic, (HhInput_t){.arm_dir = true, .joint = {[J3] = true}});
    ExpectNoMotion(&packet);
    (void)Step(&logic, (HhInput_t){0});
    assert(!logic.reverse);
}

static void TestArmStopLatchesUntilStart(void)
{
    HhLogic_t logic = EnabledLogic();

    HhPacket_t packet = Step(&logic, (HhInput_t){.arm_stop = true, .joint = {true}});
    assert(packet.flags & HH_FLAG_ARM_STOP);
    assert(packet.jog[J1] == 0);

    // 松开后仍然停止，关节键和组合键都无效，底盘不受影响
    packet = Step(&logic, (HhInput_t){.joint = {true}, .forward = 1000});
    assert(packet.flags & HH_FLAG_ARM_STOP);
    assert(packet.flags & HH_FLAG_ENABLED);
    assert(packet.jog[J1] == 0);
    assert(packet.forward == 1000);
    packet = Step(&logic, (HhInput_t){.arm_dir = true, .joint = {true}});
    assert(packet.preset == HH_PRESET_NONE);

    // LED2 快闪
    assert(HhLogic_DirLedOn(&logic, 0U));
    assert(!HhLogic_DirLedOn(&logic, HH_LED_BLINK_HALF_MS));
    assert(HhLogic_DirLedOn(&logic, 2U * HH_LED_BLINK_HALF_MS));

    // 按使能键解除
    (void)Step(&logic, (HhInput_t){0});
    packet = Step(&logic, (HhInput_t){.start = true});
    assert(!(packet.flags & HH_FLAG_ARM_STOP));
    packet = Step(&logic, (HhInput_t){.joint = {true}});
    assert(packet.jog[J1] == HH_JOG_MAX);
}

/* 停止键一直按着时按使能键：解除停止，按着的停止键不会再次触发 */
static void TestArmStopHeldThroughStart(void)
{
    HhLogic_t logic = EnabledLogic();

    HhPacket_t packet = Step(&logic, (HhInput_t){.arm_stop = true});
    assert(packet.flags & HH_FLAG_ARM_STOP);

    packet = Step(&logic, (HhInput_t){.arm_stop = true, .start = true});
    assert(!(packet.flags & HH_FLAG_ARM_STOP));

    packet = Step(&logic, (HhInput_t){.arm_stop = true, .joint = {true}});
    assert(!(packet.flags & HH_FLAG_ARM_STOP));
    assert(packet.jog[J1] == HH_JOG_MAX);
}

/* 上电时停止键就按着：不算一次按下 */
static void TestArmStopHeldAtPowerOnIgnored(void)
{
    HhLogic_t logic;
    HhLogic_Init(&logic);

    (void)Step(&logic, (HhInput_t){.arm_stop = true});
    HhPacket_t packet = Step(&logic, (HhInput_t){.arm_stop = true, .start = true});
    assert(packet.flags & HH_FLAG_ENABLED);
    assert(!(packet.flags & HH_FLAG_ARM_STOP));
}

/* 每次使能：方向回到正向，机械臂停止解除 */
static void TestStartResetsDirection(void)
{
    HhLogic_t logic = EnabledLogic();

    (void)Step(&logic, (HhInput_t){.arm_dir = true});
    (void)Step(&logic, (HhInput_t){0});
    assert(logic.reverse);

    (void)Step(&logic, (HhInput_t){.estop = true});
    (void)Step(&logic, (HhInput_t){0});
    (void)Step(&logic, (HhInput_t){.start = true});
    assert(!logic.reverse);
}

static void TestSeqIncrements(void)
{
    HhLogic_t logic;
    HhLogic_Init(&logic);
    logic.seq = 254U;

    assert(Step(&logic, (HhInput_t){0}).seq == 254U);
    assert(Step(&logic, (HhInput_t){0}).seq == 255U);
    assert(Step(&logic, (HhInput_t){0}).seq == 0U);
}

static void TestInvalidArguments(void)
{
    HhLogic_t logic = EnabledLogic();
    HhLogic_t before = logic;
    HhPacket_t packet = {.seq = 9U, .flags = HH_FLAG_ENABLED};
    HhInput_t input = {.forward = 1001};

    assert(!HhLogic_Step(&logic, &input, &packet));
    assert(packet.seq == 0U && packet.flags == 0U);
    assert(logic.seq == before.seq);

    input = (HhInput_t){0};
    assert(!HhLogic_Step(NULL, &input, &packet));
    assert(!HhLogic_Step(&logic, NULL, &packet));
    assert(!HhLogic_Step(&logic, &input, NULL));
    assert(!HhLogic_DirLedOn(NULL, 0U));
    HhLogic_Init(NULL);
}

/* 生成的包都能通过编码器的范围检查 */
static void TestPacketsEncodable(void)
{
    HhLogic_t logic = EnabledLogic();
    HhPacket_t packet = Step(&logic, (HhInput_t){.forward = -1000, .left = 1000,
                                                 .turn_cw = true, .gear = true,
                                                 .joint = {true, true, true, true, true, true}});
    uint8_t out[HH_PACKET_LEN];
    assert(HhPacket_Encode(&packet, out, sizeof(out)) == HH_PACKET_LEN);
}

int main(void)
{
    TestPowerOnDisabled();
    TestStartHeldAtPowerOnIgnored();
    TestEnableLatches();
    TestEstopDisablesAndBlocksStart();
    TestTurnAndGearPassThrough();
    TestDisabledSendsNoMotionFlags();
    TestJointJogForwardAndMultiple();
    TestDirShortPressToggles();
    TestDirComboPresets();
    TestDirWithOtherJointDoesNothing();
    TestArmStopLatchesUntilStart();
    TestArmStopHeldThroughStart();
    TestArmStopHeldAtPowerOnIgnored();
    TestStartResetsDirection();
    TestSeqIncrements();
    TestInvalidArguments();
    TestPacketsEncodable();

    puts("Handheld logic tests passed");
    return 0;
}
