//
// Created by YZH on 2026/9/25.
//
/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @brief  PID控制器定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyrightCopyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "controller.h"

#include <string.h>

/*====================================================================*/
/* ---------------------------PID 优化算法部分--------------------------- */

//梯形积分
static void f_Trapezoid_Intergral(PID_Instance *pid)
{
    pid->ITerm = pid->Ki * ((pid->Error + pid->Last_Error) / 2) * pid->dt;
}

// 变速积分
static void f_Changing_Integration_Rate(PID_Instance *pid)
{
    if (pid->Error * pid->Iout > 0)
    {
        // 积分呈累积趋势
        if (fabsf(pid->Error) <= pid->CoefB)
            return; // Full integral
        if (fabsf(pid->Error) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - fabsf(pid->Error) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PID_Instance *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (fabsf(temp_Output) > pid->MaxOut)
    {
        if (pid->Error * pid->Iout > 0) // 积分却还在累积
        {
            pid->ITerm = 0; // 当前积分项置零
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(PID_Instance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void f_Derivative_Filter(PID_Instance *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 输出滤波
static void f_Output_Filter(PID_Instance *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

// 输出限幅
static void f_Output_Limit(PID_Instance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}
/*====================================================================*/

/*====================================================================*/
/* --------------------------PID 算法外部接口-------------------------- */
/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void PID_Init(PID_Instance* pid,PID_Init_Config_s* config)
{
    memset(pid,0,sizeof(PID_Instance));
    memcpy(pid,config,sizeof(PID_Init_Config_s));
}


