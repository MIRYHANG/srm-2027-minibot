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

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PID_Calculate(PID_Instance *pid, float measure, float reference,float dt_s)
{
    if (pid == NULL || dt_s <= 0.0f)
    {
        return 0.0f;
    }
    pid->dt = dt_s;
    pid->Measure = measure;
    pid->Ref = reference;
    pid->Error = pid->Ref - pid->Measure;

    // 如果在死区外,则计算PID
    if (fabsf(pid->Error) > pid->DeadBand)
    {
        // 基本的pid计算,使用位置式
        pid->Pout = pid->Kp * pid->Error;
        pid->ITerm = pid->Ki * pid->Error * pid->dt;
        pid->Dout = pid->Kd * (pid->Error - pid->Last_Error) / pid->dt;

        // 梯形积分
        if (pid->Improve & PID_Trapezoid_Integral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 微分滤波器
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & PID_Integrate_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                         // 累加积分
        pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->Output = 0;
        pid->ITerm = 0;
    }

    // 保存当前数据,用于下次计算
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Error = pid->Error;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}

/**
 * @brief  PID清零
 * @param  pid: 指向PID结构体的指针
 * @retval 无
 */
void PID_Clear(PID_Instance *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;

    pid->Ref = 0.0f;
    pid->Error = 0.0f;
    pid->Last_Error = 0.0f;

    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;

    pid->ITerm = 0.0f;
    pid->Last_ITerm = 0.0f;

    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
}