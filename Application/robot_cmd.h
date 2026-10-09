//
// Created by YZH on 2026/10/8.
//

#ifndef XIAOSAI_ROBOT_CMD_H
#define XIAOSAI_ROBOT_CMD_H

#include <stdbool.h>
#include <stdint.h>

#include "remote_input.h"
#include "robot_def.h"

#define ROBOT_GEAR_MIN 1U
#define ROBOT_GEAR_MAX 3U

typedef struct
{
    bool enabled;   // 上电后即可清除
    bool seen_disable;  // 失能后是否已经收到过一帧"未使能"，收到后才接受使能
    bool last_gear;  // 检测按下瞬间
    uint8_t gear;    // 当前挡位
} RobotCmd_t;

typedef struct
{
    ChassisCmd_t chassis;
    ArmCmd_t arm;
} RobotCmdOutput_t;

/**
 * @brief 初始化为失能状态，1 挡
 * @note 上电和看门狗复位后都从这里开始，必须先收到一帧未使能、再收到使能才会运动
 */
void RobotCmd_Init(RobotCmd_t *robot);

/**
 * @brief 根据遥控命令和安全输入更新使能状态，生成底盘和机械臂命令
 * @param online 遥控是否在线，即 RemoteInput_GetLatest 的返回值
 * @param remote 最新遥控命令，已由 RemoteInput_Update 校验；online 为 false 时不读取
 * @param safety_active 去抖后的急停和驱动故障，非 0 表示必须停止
 * @param out 输出；不允许运动时 enabled 为 false，运动量全为零
 * @note 急停、驱动故障、操作手失能都会进入失能并锁存；遥控掉线只停止输出，不改变使能状态
 * @note 换挡键每按下一次挡位加 1，3 挡之后回到 1 挡；每次重新使能回到 1 挡
 */
void RobotCmd_Update(RobotCmd_t *robot, bool online, const RemoteCommand_t *remote,
                     uint8_t safety_active, RobotCmdOutput_t *out);


#endif //XIAOSAI_ROBOT_CMD_H
