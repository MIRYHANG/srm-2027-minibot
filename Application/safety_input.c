//
// Created by YZH on 2026/10/7.
//

#include "safety_input.h"

void SafetyInput_Init(SafetyInput_t *state)
{
    if (state == NULL)
    {
        return;
    }
    *state = (SafetyInput_t){0};
}

uint8_t SafetyInput_Update(SafetyInput_t *state, uint8_t raw)
{
    if (state == NULL)
    {
         return SAFETY_ESTOP;
    }

    for (uint8_t idx = 0;idx < SAFETY_INPUT_COUNT;idx++)
    {
        /*----------本质0000 0001向左移位------------*/
        uint8_t bit = (uint8_t)(1 << idx);
        /*----------检验raw的第idx位是否为1------------*/
        if ((raw & bit) != 0)
        {
            /*----------把active的第idx位置1------------*/
            state->active = (uint8_t)(state->active | bit);
            /*----------把idx计数器清零------------*/
            state->release_count[idx] = 0;
        }
        else if ((state->active & bit) != 0)
        {
            state->release_count[idx]++;
            if (state->release_count[idx] >= SAFETY_RELEASE_SAMPLES)
            {
                state->active = (uint8_t)(state->active & ~bit);
                state->release_count[idx] = 0U;
            }
        }
    }

    return state->active;
}

