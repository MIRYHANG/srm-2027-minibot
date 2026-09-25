/**
******************************************************************************
 * @file	 controller.h
 * @author  Wang Hongxi
 * @version V1.1.3
 * @date    2021/7/3
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _CONTROLLER_H
#define _CONTROLLER_H
#include <stdint.h>
#include <math.h>

/*========================================*/
/* PID优化调节器部分，与PID数值配置分离，通过位与方式可自行决定是否启动 */
typedef enum
{
    PID_IMPROVE_NONE =                  0        ,      // 0000 0000
    PID_Integrate_Limit =               (1U << 0),      // 0000 0001  积分限幅
    PID_Derivative_On_Measurement =     (1U << 1),      // 0000 0010  对测量值微分
    PID_Trapezoid_Integral =            (1U << 2),      // 0000 0100  梯形积分
    PID_Proportional_On_Measurement =   (1U << 3),      // 0000 1000  对测量值比例（几乎不开）
    PID_OutputFilter =                  (1U << 4),      // 0001 0000  输出低通滤波
    PID_ChangingIntegrationRate =       (1U << 5),      // 0010 0000  变速积分（防超调）
    PID_DerivativeFilter =              (1U << 6),      // 0100 0000  微分滤波
    PID_ErrorHandle =                   (1U << 7),      // 1000 0000  错误中断
} PID_Improvement_t;
/*========================================*/

/*========================================*/
/* PID 结构体（外部调用） */
typedef struct
{
    /*------------config parameter----------*/
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    /*----------improve parameter----------*/
    PID_Improvement_t Improve;
    float IntegralLimit;     // 积分限幅
    float CoefA;             // 变速积分 For Changing Integral
    float CoefB;             // 变速积分 ITerm = Error*((A-abs(error)+B)/A)  when B<|error|<A+B
    float Output_LPF_RC;     // 输出滤波器 RC = 1 / alpha
    float Derivative_LPF_RC; // 微分滤波器系数

    /*-----------for calculate--------------*/
    float Measure;
    float Last_Measure;
    float Error;
    float Last_Error;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    float dt;

} PID_Instance;

/* 用于PID初始化的结构体（内部配置参数） */
typedef struct // config parameter
{
    /*-------basic parameter------*/
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float DeadBand; // 死区

    // improve parameter
    PID_Improvement_t Improve;
    float IntegralLimit; // 积分限幅
    float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;
} PID_Init_Config_s;
/*========================================*/

/**
 * @brief 初始化PID
 *
 * @para  pid       PID指针
 * @para  config    PID初始化配置
 */
void PID_Init(PID_Instance* pid,PID_Init_Config_s* config);

/**
 * @brief 计算PID输出
 *
 * @param pid           PID实例指针
 * @param measure       反馈值
 * @param reference     设定值
 * @return float        PID计算输出
 */
float PID_Calculate(PID_Instance *pid, float measure, float reference,float dt_s);

/**
 * @brief  PID清零
 * @param  pid 指向PID结构体的指针
 */
void PID_Clear(PID_Instance *pid);


#endif