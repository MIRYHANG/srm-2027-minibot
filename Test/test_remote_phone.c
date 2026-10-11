//
// Created by YZH on 2026/10/4.
//
#include <assert.h>
#include <stddef.h>
#include <stdio.h>

#include "remote_phone.h"

/* 按字段比较，避免结构体填充字节影响测试 */
static void ExpectCommand(RemoteCommand_t actual,
                          RemoteCommand_t expected)
{
    assert(actual.forward == expected.forward);
    assert(actual.left == expected.left);
    assert(actual.turn == expected.turn);
    assert(actual.enabled == expected.enabled);
    assert(actual.stop_requested == expected.stop_requested);

    for (int idx = 0; idx < ARM_JOINT_COUNT; ++idx)
    {
        assert(actual.arm.jog[idx] == expected.arm.jog[idx]);
    }

    assert(actual.arm.preset == expected.arm.preset);
}

static void ExpectZero(RemoteCommand_t command)
{
    RemoteCommand_t zero = {0};
    ExpectCommand(command, zero);
}

/* 先给输出填入非零值，确认转换失败时确实清空了全部字段 */
static RemoteCommand_t NonzeroCommand(void)
{
    RemoteCommand_t command = {
        .forward = 1.0f,
        .left = -1.0f,
        .turn = 1.0f,
        .enabled = true,
        .stop_requested = true,
        .arm = {
            .preset = ARM_PRESET_STOW
        }
    };

    for (int idx = 0; idx < ARM_JOINT_COUNT; ++idx)
    {
        command.arm.jog[idx] = 0.5f;
    }

    return command;
}

static void ExpectOnlyJog(RemoteCommand_t command,
                          int selected, float expected)
{
    for (int idx = 0; idx < ARM_JOINT_COUNT; ++idx)
    {
        float value = idx == selected ? expected : 0.0f;
        assert(command.arm.jog[idx] == value);
    }
}

/* 1：所有关节都检查负端点、正端点和松开后的零输出 */
static void TestRightYSelectedJoint(void)
{
    RemotePhoneArm_t arm;
    RemotePhoneArm_Init(&arm);

    srm_control_state_t raw = {0};
    RemoteCommand_t command;

    for (int selected = ARM_J1;
         selected < ARM_JOINT_COUNT;
         ++selected)
    {
        arm.selected = selected;

        raw.right_y = -512;
        assert(RemotePhone_Convert(&raw, &arm, &command));
        ExpectOnlyJog(command, selected, -1.0f);

        raw.right_y = 511;
        assert(RemotePhone_Convert(&raw, &arm, &command));
        ExpectOnlyJog(command, selected, 1.0f);

        raw.right_y = 0;
        assert(RemotePhone_Convert(&raw, &arm, &command));
        ExpectOnlyJog(command, selected, 0.0f);

        assert(arm.selected == selected);
    }
}

/* 2：连续按住三帧只切换一次，松开再按才再次切换 */
static void TestUpPressEdge(void)
{
    RemotePhoneArm_t arm;
    RemotePhoneArm_Init(&arm);
    assert(RemotePhoneArm_GetSelected(&arm) == ARM_J1);

    srm_control_state_t raw = {.dpad = 1U};
    RemoteCommand_t command;

    for (int frame = 0; frame < 3; ++frame)
    {
        assert(RemotePhone_Convert(&raw, &arm, &command));
        assert(arm.selected == ARM_J2);
        assert(arm.last_dpad == 1U);
    }

    raw.dpad = 0U;
    assert(RemotePhone_Convert(&raw, &arm, &command));
    assert(arm.selected == ARM_J2);
    assert(arm.last_dpad == 0U);

    raw.dpad = 1U;
    assert(RemotePhone_Convert(&raw, &arm, &command));
    assert(arm.selected == ARM_J3);
    assert(arm.last_dpad == 1U);
}

/* 3：检查两个方向的边界回绕 */
static void TestSelectionWrap(void)
{
    RemotePhoneArm_t arm;
    RemotePhoneArm_Init(&arm);

    srm_control_state_t raw = {0};
    RemoteCommand_t command;

    arm.selected = ARM_GRIPPER;
    raw.dpad = 1U;
    assert(RemotePhone_Convert(&raw, &arm, &command));
    assert(arm.selected == ARM_J1);

    raw.dpad = 0U;
    assert(RemotePhone_Convert(&raw, &arm, &command));

    raw.dpad = 2U;
    assert(RemotePhone_Convert(&raw, &arm, &command));
    assert(arm.selected == ARM_GRIPPER);

    // 持续按住下键，同样不能重复切换
    assert(RemotePhone_Convert(&raw, &arm, &command));
    assert(arm.selected == ARM_GRIPPER);
}

/* 4：自转只由左右方向键决定，右摇杆 X 不影响自转 */
static void TestTurnMapping(void)
{
    static const int16_t right_x_values[] = {-512, 0, 511};

    static const struct
    {
        uint8_t dpad;
        float turn;
    } cases[] = {
        {3U,  1.0f},
        {4U, -1.0f},
        {0U,  0.0f}
    };

    RemotePhoneArm_t arm;
    RemotePhoneArm_Init(&arm);

    srm_control_state_t raw = {0};
    RemoteCommand_t command;

    for (size_t axis = 0;
         axis < sizeof(right_x_values) / sizeof(right_x_values[0]);
         ++axis)
    {
        raw.right_x = right_x_values[axis];

        for (size_t idx = 0;
             idx < sizeof(cases) / sizeof(cases[0]);
             ++idx)
        {
            raw.dpad = cases[idx].dpad;

            assert(RemotePhone_Convert(&raw, &arm, &command));
            assert(command.turn == cases[idx].turn);
            assert(arm.selected == ARM_J1);
        }
    }
}

/* 5：检查按钮映射，以及松开后清除上一帧请求 */
static void TestButtonMapping(void)
{
    static const struct
    {
        uint8_t buttons;
        ArmPreset_t preset;
        float gripper_jog; // X 按住夹紧为 1，Y 按住松开为 -1
    } cases[] = {
        {0x01U, ARM_PRESET_GRAB_READY,  0.0f},
        {0x02U, ARM_PRESET_STOW,        0.0f},
        {0x03U, ARM_PRESET_NONE,        0.0f},
        {0x04U, ARM_PRESET_NONE,        1.0f},
        {0x08U, ARM_PRESET_NONE,       -1.0f},
        {0x0CU, ARM_PRESET_NONE,        0.0f},
        {0x00U, ARM_PRESET_NONE,        0.0f}
    };

    RemotePhoneArm_t arm;
    RemotePhoneArm_Init(&arm);

    srm_control_state_t raw = {0};
    RemoteCommand_t command;

    for (size_t idx = 0;
         idx < sizeof(cases) / sizeof(cases[0]);
         ++idx)
    {
        raw.buttons = cases[idx].buttons;
        assert(RemotePhone_Convert(&raw, &arm, &command));

        RemoteCommand_t expected = {0};
        expected.arm.preset = cases[idx].preset;
        expected.arm.jog[ARM_GRIPPER] = cases[idx].gripper_jog;

        ExpectCommand(command, expected);
    }
}

/* 5b：选中夹爪时右摇杆也能点动夹爪，X/Y 优先于右摇杆 */
static void TestGripperKeysOverrideStick(void)
{
    RemotePhoneArm_t arm;
    RemotePhoneArm_Init(&arm);
    arm.selected = ARM_GRIPPER;

    srm_control_state_t raw = {.right_y = 511};
    RemoteCommand_t command;

    // 只推右摇杆：夹爪按摇杆点动
    assert(RemotePhone_Convert(&raw, &arm, &command));
    ExpectOnlyJog(command, ARM_GRIPPER, 1.0f);

    // 摇杆推向夹紧，同时按 Y：按松开处理
    raw.buttons = 0x08U;
    assert(RemotePhone_Convert(&raw, &arm, &command));
    ExpectOnlyJog(command, ARM_GRIPPER, -1.0f);

    // X、Y 同时按：夹爪不动，摇杆也不起作用
    raw.buttons = 0x0CU;
    assert(RemotePhone_Convert(&raw, &arm, &command));
    ExpectOnlyJog(command, ARM_GRIPPER, 0.0f);
}

/* 6：检查 S1 和 S2 的四种组合 */
static void TestSwitchMapping(void)
{
    static const struct
    {
        uint8_t switches;
        bool enabled;
        bool stop;
    } cases[] = {
        {0x00U, false, false},
        {0x01U, true,  false},
        {0x02U, false, true },
        {0x03U, true,  true }
    };

    RemotePhoneArm_t arm;
    RemotePhoneArm_Init(&arm);

    srm_control_state_t raw = {0};
    RemoteCommand_t command;

    for (size_t idx = 0;
         idx < sizeof(cases) / sizeof(cases[0]);
         ++idx)
    {
        raw.switches = cases[idx].switches;
        assert(RemotePhone_Convert(&raw, &arm, &command));

        RemoteCommand_t expected = {0};
        expected.enabled = cases[idx].enabled;
        expected.stop_requested = cases[idx].stop;

        ExpectCommand(command, expected);
    }
}

/* 拒绝输入时，输出必须清零，关节选择和按键历史必须保留 */
static void ExpectRejected(const srm_control_state_t *raw,
                           RemotePhoneArm_t *arm)
{
    RemotePhoneArm_t before = *arm;
    RemoteCommand_t command = NonzeroCommand();

    assert(!RemotePhone_Convert(raw, arm, &command));
    ExpectZero(command);
    assert(arm->selected == before.selected);
    assert(arm->last_dpad == before.last_dpad);
}

/* 7：逐一检查四个轴的上下越界、非法方向键和按钮高四位 */
static void TestInvalidFrame(void)
{
    static const srm_control_state_t cases[] = {
        {.left_x  = -513, .dpad = 1U},
        {.left_x  =  512, .dpad = 1U},
        {.left_y  = -513, .dpad = 1U},
        {.left_y  =  512, .dpad = 1U},
        {.right_x = -513, .dpad = 1U},
        {.right_x =  512, .dpad = 1U},
        {.right_y = -513, .dpad = 1U},
        {.right_y =  512, .dpad = 1U},
        {.dpad = 5U},
        {.buttons = 0x10U, .dpad = 1U},
        {.buttons = 0x20U, .dpad = 1U},
        {.buttons = 0x40U, .dpad = 1U},
        {.buttons = 0x80U, .dpad = 1U}
    };

    RemotePhoneArm_t arm;
    RemotePhoneArm_Init(&arm);
    arm.selected = ARM_J3;
    arm.last_dpad = 4U;

    for (size_t idx = 0;
         idx < sizeof(cases) / sizeof(cases[0]);
         ++idx)
    {
        ExpectRejected(&cases[idx], &arm);
    }
}

/* 8：检查空指针和非法关节编号 */
static void TestInvalidArguments(void)
{
    srm_control_state_t raw = {.dpad = 1U};
    RemotePhoneArm_t arm;
    RemotePhoneArm_Init(&arm);

    RemoteCommand_t command = NonzeroCommand();
    assert(!RemotePhone_Convert(&raw, NULL, &command));
    ExpectZero(command);

    arm.selected = -1;
    ExpectRejected(&raw, &arm);

    arm.selected = ARM_JOINT_COUNT;
    ExpectRejected(&raw, &arm);

    // 补充 raw 和 command 空指针检查
    RemotePhoneArm_Init(&arm);
    ExpectRejected(NULL, &arm);

    RemotePhoneArm_t before = arm;
    assert(!RemotePhone_Convert(&raw, &arm, NULL));
    assert(arm.selected == before.selected);
    assert(arm.last_dpad == before.last_dpad);
}

/* 组装一帧并逐字节输入，只有最后一个字节应完成命令更新 */
static void FeedControlFrame(srm_parser_t *parser,
                             RemoteInput_t *input,
                             RemotePhoneArm_t *arm,
                             const srm_control_state_t *raw,
                             uint8_t sequence,
                             uint32_t now_ms)
{
    uint8_t bytes[SRM_MAX_FRAME];

    size_t length = srm_build_control(bytes, sizeof(bytes),
                                      sequence, raw);
    assert(length > 0U);
    assert(length <= sizeof(bytes));

    for (size_t idx = 0; idx < length; ++idx)
    {
        bool updated = RemotePhone_ProcessByte(
            parser, input, arm, bytes[idx], now_ms);

        bool is_last_byte = idx + 1U == length;
        assert(updated == is_last_byte);
    }
}

/* 9：验证组帧、逐字节解析、重新上锁和安全命令读取 */
static void TestProcessByteAndRearm(void)
{
    srm_parser_t parser;
    srm_parser_init(&parser);

    RemoteInput_t input;
    RemoteInput_Init(&input);
    RemoteInput_Select(&input, REMOTE_SOURCE_PHONE);

    RemotePhoneArm_t arm;
    RemotePhoneArm_Init(&arm);

    srm_control_state_t raw = {
        .left_x = -256,
        .left_y = 511,
        .right_x = 511,
        .right_y = -512,
        .buttons = 0x05U, // A + X
        .switches = 0U,  // S1 关闭
        .dpad = 1U       // 上键将 J1 切换为 J2
    };

    assert(!input.phone.armed);

    FeedControlFrame(&parser, &input, &arm, &raw, 1U, 100U);

    assert(input.phone.armed);
    assert(input.phone.has_valid_frame);
    assert(input.phone.last_valid_ms == 100U);
    assert(arm.selected == ARM_J2);
    ExpectZero(RemoteInput_GetSafe(&input, 101U, 200U));

    // 第二帧打开 S1，并按左方向键请求逆时针自转
    raw.switches = 0x01U;
    raw.dpad = 3U;

    FeedControlFrame(&parser, &input, &arm, &raw, 2U, 120U);

    RemoteCommand_t expected = {
        .forward = 1.0f,
        .left = -0.5f,
        .turn = 1.0f,
        .enabled = true,
        .arm = {
            .jog = {[ARM_J2] = -1.0f, [ARM_GRIPPER] = 1.0f},
            .preset = ARM_PRESET_GRAB_READY
        }
    };

    assert(input.phone.armed);
    assert(input.phone.last_valid_ms == 120U);
    assert(arm.selected == ARM_J2);
    assert(arm.last_dpad == 3U);

    ExpectCommand(RemoteInput_GetSafe(&input, 121U, 200U),
                  expected);
}

int main(void)
{
    TestRightYSelectedJoint();
    TestUpPressEdge();
    TestSelectionWrap();
    TestTurnMapping();
    TestButtonMapping();
    TestGripperKeysOverrideStick();
    TestSwitchMapping();
    TestInvalidFrame();
    TestInvalidArguments();
    TestProcessByteAndRearm();

    puts("Remote phone tests passed");
    return 0;
}