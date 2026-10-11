//
// Created by YZH on 2026/10/10.
//

#ifndef XIAOSAI_CHASSIS_DRIVE_H
#define XIAOSAI_CHASSIS_DRIVE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "mecanum.h"
#include "robot_def.h"

#include <math.h>

// TODO: 数值待定，先放个10，20，30占位
#define CHASSIS_GEAR1_MAX_RPM       10.0f
#define CHASSIS_GEAR2_MAX_RPM       20.0f
#define CHASSIS_GEAR3_MAX_RPM       30.0f

// TODO: 依旧数值待定
/*------------自转的转动系数-----------*/
#define CHASSIS_TURN_SCALE          1.0f

// TODO: 依旧数值待定
/*----------轮速每秒最多变化多少rpm-----------*/
#define CHASSIS_WHEEL_RPM_PER_S     60.0f

typedef struct {
    MecanumWheelRpm_t current;
} ChassisRamp_t;

/**
 * @brief 取某个挡位的最大轮速
 * @return 挡位 1～3 返回对应转速，其他值返回 0
 */
float ChassisDrive_GearMaxRpm(uint8_t gear);

/**
 * @brief 把底盘命令换算成四轮目标转速
 * @param cmd 底盘命令；forward、left、turn 范围 -1～1
 * @param out 四轮目标转速，单位 rpm，绝对值不超过当前挡位的最大转速
 * @return 换算成功或未使能返回 true；输入非法返回 false
 * @note 推满摇杆时最快那个轮子正好等于挡位转速，推一半就是一半
 */
bool ChassisDrive_ToWheelRpm(const ChassisCmd_t *cmd, MecanumWheelRpm_t *out);


/**
 * @brief 把斜坡的当前转速清零
 */
void ChassisRamp_Reset(ChassisRamp_t *ramp);

/**
 * @brief 让四轮转速以有限的加速度跟随目标
 * @param target 本周期的四轮目标转速
 * @param dt_s 距上次调用的时间，单位为秒
 * @param out 本周期应输出的四轮转速，允许和 target 指向同一个对象
 * @return 成功返回 true；参数非法返回 false，此时 ramp 不变，out 非空时写入上一次的转速
 * @note 改写自 WPILib SlewRateLimiter，并按 MecanumDriveWheelVelocities::Desaturate 的思路改成四轮同步
 */
bool ChassisRamp_Step(ChassisRamp_t *ramp, const MecanumWheelRpm_t *target,
                      float dt_s, MecanumWheelRpm_t *out);

#endif //XIAOSAI_CHASSIS_DRIVE_H
