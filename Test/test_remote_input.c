#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "remote_input.h"

static RemoteCommand_t EnabledCommand(void)
{
    return (RemoteCommand_t){
        .forward = 0.5f,
        .left = -0.25f,
        .turn = 0.1f,
        .enabled = true
    };
}

static RemoteCommand_t DisabledCommand(void)
{
    RemoteCommand_t command = EnabledCommand();
    command.enabled = false;
    return command;
}

static void ExpectZero(RemoteCommand_t command)
{
    assert(command.forward == 0.0f);
    assert(command.left == 0.0f);
    assert(command.turn == 0.0f);
    assert(!command.enabled);
    assert(!command.stop_requested);
}

static void ExpectCommand(RemoteCommand_t actual,
                          RemoteCommand_t expected)
{
    assert(actual.forward == expected.forward);
    assert(actual.left == expected.left);
    assert(actual.turn == expected.turn);
    assert(actual.enabled == expected.enabled);
    assert(actual.stop_requested == expected.stop_requested);
}

static void TestStartupLock(void)
{
    RemoteInput_t input;
    RemoteInput_Init(&input);
    assert(!input.phone.armed);
    assert(!input.handheld.armed);

    RemoteInput_Select(&input, REMOTE_SOURCE_PHONE);

    RemoteCommand_t enabled = EnabledCommand();
    RemoteCommand_t disabled = DisabledCommand();

    // 复位后第一帧仍然使能，不能立即运动
    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &enabled, 100U));
    assert(!input.phone.armed);
    ExpectZero(RemoteInput_GetSafe(&input, 110U, 200U));

    // 先收到关闭使能的有效帧，再收到使能帧，才允许输出
    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &disabled, 120U));
    assert(input.phone.armed);
    ExpectZero(RemoteInput_GetSafe(&input, 130U, 200U));

    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &enabled, 140U));
    ExpectCommand(RemoteInput_GetSafe(&input, 150U, 200U), enabled);
}

static void TestTimeoutKeepsArmed(void)
{
    RemoteInput_t input;
    RemoteInput_Init(&input);
    RemoteInput_Select(&input, REMOTE_SOURCE_PHONE);

    RemoteCommand_t enabled = EnabledCommand();
    RemoteCommand_t disabled = DisabledCommand();

    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &disabled, 100U));
    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &enabled, 110U));

    // 距离上一帧超过 200 ms
    ExpectZero(RemoteInput_GetSafe(&input, 311U, 200U));
    assert(input.phone.armed);

    // 恢复通信后无需再次发送关闭使能帧
    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &enabled, 400U));
    ExpectCommand(RemoteInput_GetSafe(&input, 401U, 200U), enabled);
}

static void TestSwitchRequiresRearm(void)
{
    RemoteInput_t input;
    RemoteInput_Init(&input);
    RemoteInput_Select(&input, REMOTE_SOURCE_PHONE);

    RemoteCommand_t enabled = EnabledCommand();
    RemoteCommand_t disabled = DisabledCommand();

    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &disabled, 100U));
    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &enabled, 110U));
    ExpectCommand(RemoteInput_GetSafe(&input, 111U, 200U), enabled);

    RemoteInput_Select(&input, REMOTE_SOURCE_HANDHELD);
    assert(!input.handheld.armed);

    RemoteInput_Select(&input, REMOTE_SOURCE_PHONE);
    assert(!input.phone.armed);

    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &enabled, 120U));
    ExpectZero(RemoteInput_GetSafe(&input, 121U, 200U));

    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &disabled, 130U));
    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &enabled, 140U));
    ExpectCommand(RemoteInput_GetSafe(&input, 141U, 200U), enabled);
}

static void TestStopRequest(void)
{
    RemoteInput_t input;
    RemoteInput_Init(&input);
    RemoteInput_Select(&input, REMOTE_SOURCE_PHONE);

    RemoteCommand_t enabled = EnabledCommand();
    RemoteCommand_t disabled = DisabledCommand();
    RemoteCommand_t stopped = enabled;
    stopped.stop_requested = true;

    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &disabled, 100U));
    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &stopped, 110U));
    ExpectZero(RemoteInput_GetSafe(&input, 111U, 200U));
    assert(input.phone.armed);
}

static void TestInvalidFrameCannotArm(void)
{
    RemoteInput_t input;
    RemoteInput_Init(&input);
    RemoteInput_Select(&input, REMOTE_SOURCE_PHONE);

    RemoteCommand_t disabled = DisabledCommand();
    disabled.forward = NAN;

    assert(!RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                               &disabled, 100U));
    assert(!input.phone.armed);

    RemoteCommand_t enabled = EnabledCommand();
    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &enabled, 110U));
    ExpectZero(RemoteInput_GetSafe(&input, 111U, 200U));
}

/* 只有当前来源未超时、关闭使能的有效帧才算操作手已关闭使能 */
static void TestOperatorDisabled(void)
{
    RemoteInput_t input;
    RemoteInput_Init(&input);

    // 未选择来源
    assert(!RemoteInput_OperatorDisabled(&input, 100U, 200U));

    RemoteInput_Select(&input, REMOTE_SOURCE_PHONE);
    // 还没有收到任何帧
    assert(!RemoteInput_OperatorDisabled(&input, 100U, 200U));

    // 未选中的来源发来的关闭帧不算
    RemoteCommand_t disabled = DisabledCommand();
    assert(RemoteInput_Update(&input, REMOTE_SOURCE_HANDHELD,
                              &disabled, 100U));
    assert(!RemoteInput_OperatorDisabled(&input, 101U, 200U));

    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &disabled, 100U));
    assert(RemoteInput_OperatorDisabled(&input, 300U, 200U));
    // 超时后零命令不能被当作操作手关闭了使能
    assert(!RemoteInput_OperatorDisabled(&input, 301U, 200U));

    RemoteCommand_t enabled = EnabledCommand();
    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &enabled, 400U));
    assert(!RemoteInput_OperatorDisabled(&input, 401U, 200U));

    assert(!RemoteInput_OperatorDisabled(NULL, 401U, 200U));
    assert(!RemoteInput_OperatorDisabled(&input, 401U, 0U));
}

int main(void)
{
    TestStartupLock();
    TestTimeoutKeepsArmed();
    TestSwitchRequiresRearm();
    TestStopRequest();
    TestInvalidFrameCannotArm();
    TestOperatorDisabled();

    puts("Remote input tests passed");
    return 0;
}