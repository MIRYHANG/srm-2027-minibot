//
// Created by YZH on 2026/10/7.
//
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "safety_input.h"

static SafetyInput_t InitState(void)
{
    SafetyInput_t state;
    SafetyInput_Init(&state);
    assert(SafetyInput_Update(&state, 0U) == 0U);
    return state;
}

/* 连续送入 count 次无效采样 */
static uint8_t FeedReleased(SafetyInput_t *state, uint32_t count)
{
    uint8_t active = 0U;
    for (uint32_t idx = 0U; idx < count; ++idx)
    {
        active = SafetyInput_Update(state, 0U);
    }
    return active;
}

/* 每个输入有效时立刻生效 */
static void TestAssertImmediate(void)
{
    static const uint8_t inputs[] = {
        SAFETY_ESTOP, SAFETY_FAULT_FL, SAFETY_FAULT_FR, SAFETY_FAULT_RL, SAFETY_FAULT_RR
    };

    for (size_t idx = 0U; idx < sizeof(inputs) / sizeof(inputs[0]); ++idx)
    {
        SafetyInput_t state = InitState();
        assert(SafetyInput_Update(&state, inputs[idx]) == inputs[idx]);
    }
}

/* 松开后要连续 SAFETY_RELEASE_SAMPLES 次无效才解除 */
static void TestReleaseDebounce(void)
{
    SafetyInput_t state = InitState();
    (void)SafetyInput_Update(&state, SAFETY_ESTOP);

    assert(FeedReleased(&state, SAFETY_RELEASE_SAMPLES - 1U) == SAFETY_ESTOP);
    assert(FeedReleased(&state, 1U) == 0U);
}

/* 抖动会让解除计数重新开始 */
static void TestBounceRestartsRelease(void)
{
    SafetyInput_t state = InitState();
    (void)SafetyInput_Update(&state, SAFETY_ESTOP);

    (void)FeedReleased(&state, SAFETY_RELEASE_SAMPLES - 1U);
    assert(SafetyInput_Update(&state, SAFETY_ESTOP) == SAFETY_ESTOP);
    assert(FeedReleased(&state, SAFETY_RELEASE_SAMPLES - 1U) == SAFETY_ESTOP);
    assert(FeedReleased(&state, 1U) == 0U);
}

/* 多个输入各自计数，一个解除不影响另一个 */
static void TestInputsIndependent(void)
{
    SafetyInput_t state = InitState();
    (void)SafetyInput_Update(&state, SAFETY_ESTOP | SAFETY_FAULT_FL);

    for (uint32_t idx = 0U; idx < SAFETY_RELEASE_SAMPLES; ++idx)
    {
        (void)SafetyInput_Update(&state, SAFETY_FAULT_FL);
    }
    assert(SafetyInput_Update(&state, SAFETY_FAULT_FL) == SAFETY_FAULT_FL);
}

/* 掩码以外的位不产生停止 */
static void TestUnknownBitsIgnored(void)
{
    SafetyInput_t state = InitState();
    assert(SafetyInput_Update(&state, (uint8_t)~SAFETY_ALL_MASK) == 0U);
}

static void TestNullState(void)
{
    SafetyInput_Init(NULL);
    assert(SafetyInput_Update(NULL, 0U) == SAFETY_ESTOP);
}

int main(void)
{
    TestAssertImmediate();
    TestReleaseDebounce();
    TestBounceRestartsRelease();
    TestInputsIndependent();
    TestUnknownBitsIgnored();
    TestNullState();

    puts("Safety input tests passed");
    return 0;
}