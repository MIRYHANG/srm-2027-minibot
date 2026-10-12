//
// Created by YZH on 2026/10/11.
//

#include "hh_logic.h"

#include <math.h>

#define JOINT_STOW_KEY          0U // 方向键 + J1：回到 STOW
#define JOINT_READY_KEY         1U // 方向键 + J2：进入 READY

void HhLogic_Init(HhLogic_t *logic)
{
    if (logic == NULL)
    {
        return;
    }

    /*---按键历史视为按下，上电时就按着的键不会被当成一次按下---*/
    *logic = (HhLogic_t){
        .last_start = true,
        .last_arm_stop = true,
    };
}

static bool AnyJointPressed(const HhInput_t *input)
{
    for (size_t idx = 0;idx < HH_JOINT_COUNT;++idx)
    {
        if (input->joint[idx])
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief 使能和失能锁存：失能键优先，使能键按下沿才生效
 */
static void UpdateEnable(HhLogic_t *logic, const HhInput_t *input)
{
    /* ------  input 表示当前状态，logic 表示锁存状态 ---------- */
    bool start_pressed = input->start && !logic->last_start;
    logic->last_start = input->start;

    if (input->estop)
    {
        logic->enabled = false;
        return;
    }
    if (start_pressed)
    {
        logic->enabled = true;
        logic->reverse = false;
        logic->arm_stopped = false;
    }
}

/**
 * @brief 机械臂停止锁存：停止键按下沿置位，只能由使能键清除
 */
static void UpdateArmStop(HhLogic_t *logic, const HhInput_t *input)
{
    if (input->arm_stop && !logic->last_arm_stop)
    {
        logic->arm_stopped = true;
    }
    logic->last_arm_stop = input->arm_stop;
}

/**
 * @brief 方向键：按住期间按过关节键就算组合键，否则松开时切换方向
 */
static void UpdateDirection(HhLogic_t *logic, const HhInput_t *input)
{
    if (input->arm_dir)
    {
        if (!logic->dir_held)
        {
            logic->dir_held = true;
            logic->dir_combo = false;
        }

        if (AnyJointPressed(input))
        {
            logic->dir_combo = true;
        }
        return;
    }

    if (logic->dir_held)
    {
        logic->dir_held = false;
        if (!logic->dir_combo)
        {
            logic->reverse = !logic->reverse;
        }
    }
}

/**
 * @brief 按住方向键时，J1、J2 只用来选预设；同时按住两个不发预设
 */
static uint8_t ComboPreset(const HhInput_t *input)
{
    bool stow = input->joint[JOINT_STOW_KEY];
    bool ready = input->joint[JOINT_READY_KEY];

    if (stow && !ready)
    {
        return HH_PRESET_STOW;
    }
    if (ready && !stow)
    {
        return HH_PRESET_READY;
    }
    return HH_PRESET_NONE;
}

/**
 * @brief 填写机械臂部分：停止时全零；按住方向键时只发预设；否则按当前方向点动
 */
static void FillArm(const HhLogic_t *logic, const HhInput_t *input, HhPacket_t *packet)
{
    if (logic->arm_stopped)
    {
        return;
    }

    if (logic->dir_held)
    {
        packet->preset = ComboPreset(input);
        return;
    }

    int8_t speed = logic->reverse ? (int8_t)-HH_JOG_MAX : (int8_t)HH_JOG_MAX;
    for (size_t idx = 0; idx < HH_JOINT_COUNT; ++idx)
    {
        packet->jog[idx] = input->joint[idx] ? speed : (int8_t)0;
    }
}

/**
 * @brief 检查摇杆值是否在 -HH_AXIS_MAX～HH_AXIS_MAX 内
 */
static bool AxisValid(int16_t value)
{
    return value >= -HH_AXIS_MAX && value <= HH_AXIS_MAX;
}

bool HhLogic_Step(HhLogic_t *logic, const HhInput_t *input, HhPacket_t *packet)
{
    if (packet == NULL)
    {
        return false;
    }

    *packet = (HhPacket_t){0};

    if (logic == NULL || input == NULL ||
        !AxisValid(input->forward) || !AxisValid(input->left))
    {
        return false;
    }

    UpdateEnable(logic, input);
    UpdateArmStop(logic, input);
    UpdateDirection(logic, input);

    HhPacket_t next = {.seq = logic->seq};
    logic->seq++;

    if (input->estop)
    {
        next.flags |= HH_FLAG_ESTOP;
    }

    if (logic->enabled)
    {
        next.flags |= HH_FLAG_ENABLED;
        next.flags |= input->gear ? HH_FLAG_GEAR : 0U;
        next.flags |= input->turn_ccw ? HH_FLAG_TURN_CCW : 0U;
        next.flags |= input->turn_cw ? HH_FLAG_TURN_CW : 0U;
        next.flags |= logic->arm_stopped ? HH_FLAG_ARM_STOP : 0U;
        next.forward = input->forward;
        next.left = input->left;
        FillArm(logic, input, &next);
    }

    *packet = next;
    return true;
}

bool HhLogic_DirLedOn(const HhLogic_t *logic, uint32_t now_ms)
{
    if (logic == NULL)
    {
        return false;
    }

    if (logic->arm_stopped)
    {
        return ((now_ms / HH_LED_BLINK_HALF_MS) % 2U) == 0U;
    }

    return logic->reverse;
}