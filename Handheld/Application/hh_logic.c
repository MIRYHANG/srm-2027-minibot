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
    if (input->arm_stop && !logic->arm_stopped)
    {
        logic->arm_stopped = true;
    }
    logic->last_arm_stop = input->arm_stop;
}