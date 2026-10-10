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

// TODO: 数值待定，先放个10，20，30占位
#define CHASSIS_GEAR1_MAX_RPM   10.0f
#define CHASSIS_GEAR2_MAX_RPM   20.0f
#define CHASSIS_GEAR3_MAX_RPM   30.0f

// TODO: 依旧数值待定
/*------------自转的转动系数-----------*/
#define CHASSIS_TURN_SCALE      1.0f

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

#endif //XIAOSAI_CHASSIS_DRIVE_H
