//
// Created by YZH on 2026/10/12.
//

#include "hh_input.h"

/*---------------------按键消抖-------------------*/
void HhDebounce_Init(HhDebounce_t *debounce,uint16_t initial)
{
    if (debounce == NULL)
    {
        return;
    }

    *debounce = (HhDebounce_t){
        .stable = (uint16_t)(initial & HH_BTN_VALID_MASK)
    };
}

uint16_t HhDebounce_Update(HhDebounce_t *debounce, uint16_t raw)
{
    if (debounce == NULL)
    {
        return HH_BTN_BIT(HH_BTN_ESTOP);
    }

    for (uint8_t btn = 0;btn < HH_BTN_COUNT;++btn)
    {
        uint16_t bit = HH_BTN_BIT(btn);
        bool pressed = (raw & bit) != 0;
        bool stable = (debounce->stable & bit) != 0;

        /*---和当前状态一致计数清零---*/
        if (pressed == stable)
        {
            debounce->count[btn] = 0;
            continue;
        }

        /*------失能键按下不等待，立即生效------*/
        if (pressed && (HH_BTN_IMMEDIATE_MASK & bit) != 0)
        {
            debounce->stable |= bit;
            debounce->count[btn] = 0;
            continue;
        }

        debounce->count[btn]++;
        if (debounce->count[btn] >= HH_DEBOUNCE_SAMPLE)
        {
            debounce->stable = (uint16_t)(debounce->stable ^ bit);
            debounce->count[btn] = 0;
        }
    }
    return debounce->stable;
}

/*---------------- 摇杆换算 ----------------*/

void HhAxis_Init(HhAxis_t *axis, bool invert)
{
    if (axis == NULL)
    {
        return;
    }

    axis->center = HH_STICK_CENTER;
    axis->invert = invert;
}

bool HhAxis_Calibrate(HhAxis_t *axis, const uint16_t *samples, size_t count)
{
    if (axis == NULL || samples == NULL || count == 0U)
    {
        return false;
    }

    uint32_t sum = 0;
    for (size_t idx = 0;idx < count;++idx)
    {
        sum += samples[idx];
    }
    uint32_t average = (uint32_t)(sum / count);

    uint32_t low = HH_STICK_CENTER - HH_STICK_CENTER_TOL;
    uint32_t high = HH_STICK_CENTER + HH_STICK_CENTER_TOL;
    if (average < low || average > high)
    {
        return false;
    }

    axis->center = (uint16_t)average;
    return true;
}

int16_t HhAxis_Normalize(const HhAxis_t *axis, uint16_t raw)
{
    if (axis == NULL)
    {
        return 0;
    }

    if (raw > HH_ADC_MAX)
    {
        raw = HH_ADC_MAX;
    }

    int32_t offset = (int32_t)raw - (int32_t)axis->center;
    int32_t deadzone = (int32_t)HH_STICK_DEADZONE;
    int32_t value = 0;

    if (offset > deadzone)
    {
        int32_t span = (int32_t)(HH_ADC_MAX - HH_STICK_EDGE) - (int32_t)axis->center - deadzone;
        value = span > 0 ? (offset - deadzone) * HH_AXIS_MAX / span : 0;
    }
    else if (offset < -deadzone)
    {
        int32_t span = (int32_t)axis->center - (int32_t)HH_STICK_EDGE - deadzone;
        value = span > 0 ? (offset + deadzone) * HH_AXIS_MAX / span : 0;
    }

    if (value > HH_AXIS_MAX)
    {
        value = HH_AXIS_MAX;
    }
    else if (value < -HH_AXIS_MAX)
    {
        value = -HH_AXIS_MAX;
    }

    return (int16_t)(axis->invert ? -value : value);
}

/*---------------- 组装输入 ----------------*/

static bool Pressed(uint16_t buttons, HhButton_t btn)
{
    return (buttons & HH_BTN_BIT(btn)) != 0U;
}

void HhInput_Build(uint16_t buttons, int16_t forward, int16_t left, HhInput_t *out)
{
    if (out == NULL)
    {
        return;
    }

    HhInput_t next = {
        .start = Pressed(buttons, HH_BTN_START),
        .estop = Pressed(buttons, HH_BTN_ESTOP),
        .gear = Pressed(buttons, HH_BTN_GEAR),
        .turn_ccw = Pressed(buttons, HH_BTN_TURN_CCW),
        .turn_cw = Pressed(buttons, HH_BTN_TURN_CW),
        .arm_dir = Pressed(buttons, HH_BTN_ARM_DIR),
        .arm_stop = Pressed(buttons, HH_BTN_ARM_STOP),
        .forward = forward,
        .left = left
    };

    for (size_t idx = 0; idx < HH_JOINT_COUNT; ++idx)
    {
        next.joint[idx] = Pressed(buttons, (HhButton_t)(HH_BTN_J1 + idx));
    }

    *out = next;
}