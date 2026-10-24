//
// Created by YZH on 2026/10/24.
//

#ifndef XIAOSAI_HEADING_HOLD_H
#define XIAOSAI_HEADING_HOLD_H

#include <stdbool.h>

#include "robot_def.h"

// TODO：实车调参。增益为 0 时不修正，先确认 IMU 方向再逐步加大
// 两个增益允许编译时覆盖，主机测试用非零值检查修正方向
#ifndef HEADING_KP_RPM_PER_DEG
#define HEADING_KP_RPM_PER_DEG  0.0f  // 航向每偏 1°，加多少 rpm 的自转修正
#endif
#ifndef HEADING_KD_RPM_PER_DPS
#define HEADING_KD_RPM_PER_DPS  0.0f  // 角速度每 1 °/s，减多少 rpm，抑制来回摆
#endif
#define HEADING_MAX_RPM         10.0f // 修正量上限，单位 rpm
#define HEADING_SETTLE_S        0.3f  // 松开自转键后等车身停转，再锁定航向
#define HEADING_MOVE_DEADBAND   0.05f // 前后、左右都小于它视为停车，不修正

typedef struct
{
    bool locked;      // 已锁定目标航向
    float target_deg; // 要保持的航向
    float settle_s;   // 距离锁定还要等的时间
} HeadingHold_t;

/**
 * @brief 清除锁定，下次平移时重新锁定当前航向
 * @note 停车、失能或 IMU 无效时调用
 */
void HeadingHold_Reset(HeadingHold_t *hold);

/**
 * @brief 平移且没按自转键时，输出保持航向所需的自转量
 * @param cmd 底盘命令；turn 不为 0 表示操作手在自转
 * @param imu_valid IMU 已完成标定且数据未过期
 * @param yaw_deg 当前航向，逆时针为正
 * @param rate_dps 当前角速度，逆时针为正
 * @param dt_s 距上次调用的时间，单位 s
 * @return 本周期使用的 turn，范围 -1～1：操作手自转时原样返回，否则为修正量
 * @note 只在平移时修正；停车时返回 0，车被推动也不会原地扭回去
 */
float HeadingHold_Update(HeadingHold_t *hold, const ChassisCmd_t *cmd, bool imu_valid,
                         float yaw_deg, float rate_dps, float dt_s);

#endif //XIAOSAI_HEADING_HOLD_H
