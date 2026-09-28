//
// Created by YZH on 2026/9/27
//
#include <assert.h>
#include "remote_input.h"

int main(void)
{
    RemoteInput_t input;
    RemoteInput_Init(&input);

    RemoteCommand_t result = RemoteInput_GetSafe(&input, 100U, 200U);
    assert(result.forward == 0.0f);
    assert(result.left == 0.0f);
    assert(result.turn == 0.0f);

    RemoteInput_Select(&input, REMOTE_SOURCE_PHONE);

    RemoteCommand_t phone_command = {
        .forward = 0.5f,
        .left = -0.25f,
        .turn = 0.1f,
        .enabled = true
    };

    assert(RemoteInput_Update(&input, REMOTE_SOURCE_PHONE,
                              &phone_command, 100U));

    result = RemoteInput_GetSafe(&input, 150U, 200U);
    assert(result.forward == 0.5f);
    assert(result.left == -0.25f);
    assert(result.turn == 0.1f);

    return 0;
}
