//
// Created by YZH on 2026/10/3.
//

#include "arm_motion.h"
#include <math.h>
#include <stddef.h>

#include "arm_calib.h"

/*--------------------------定义标定关节的变量，并计算步幅-----------------------------------*/
static bool StepJoint(int idx, float current, float target, float dt_s, float *next)
{
    const ArmJointCalib_t *calib = ArmCalib_Get(idx);
    float from;
    float to;

    if (calib == NULL || !ArmCalib_Clamp(idx,current,&from) || !ArmCalib_Clamp(idx,target,&to))
    {
        return false;
    }

    float max_step = calib->max_speed_per_s * dt_s;
    float distance = to - from;

    /*--------剩下的距离一步就能走完---------*/
    if (fabsf(distance) <= max_step)
    {
        /*--------直接到达目标---------*/
        *next = to;
    }
    /*--------距离较远---------*/
    else if (distance > 0.0f)
    {
        /*--------往大的方向走满一步---------*/
        *next = from + max_step;
    }
    else
    {
        /*--------往小的方向走满一步---------*/
        *next = from - max_step;
    }

    return ArmCalib_Clamp(idx,*next,next);
}

bool ArmMotion_Step(const ArmPose_t *current,
                    const ArmPose_t *target,
                    float dt_s,
                    ArmPose_t *out)
{
    if (current == NULL || target == NULL || out == NULL ||
        !isfinite(dt_s) || dt_s <= 0.0f)
    {
        return false;
    }

    ArmPose_t next = {0};

    for (int idx = 0;idx < ARM_JOINT_COUNT;idx++)
    {
        if (!StepJoint(idx, current->joint[idx], target->joint[idx],
                       dt_s, &next.joint[idx]))
        {
            return false;
        }
    }

    *out = next;
    return true;
}