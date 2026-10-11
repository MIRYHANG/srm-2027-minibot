//
// Created by YZH on 2026/10/11.
//

#include "hh_logic.h"

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
    
}