//
// Created by YZH on 2026/9/27
//

#ifndef XIAOSAI_REMOTE_PHONE_H
#define XIAOSAI_REMOTE_PHONE_H

#include <stdbool.h>

#include "remote_input.h"
#include "srm_protocol.h"

typedef struct
{
    int selected;        // 当前关节：ARM_J1～ARM_GRIPPER
    uint8_t last_dpad;   // 上一帧方向键，用于识别按下沿
} RemotePhoneArm_t;

/** 初始化手机端关节选择，默认选中 J1 */
void RemotePhoneArm_Init(RemotePhoneArm_t *arm);

/** 返回选中关节；参数或状态无效时返回 -1 */
int RemotePhoneArm_GetSelected(const RemotePhoneArm_t *arm);

/**
 * @brief 将已解码的手机 CONTROL 状态转换为统一遥控命令
 * @param raw 输入的协议状态，四个摇杆轴均须在 [-512, 511] 范围内
 * @param arm 当前选中关节及上一帧方向键，转换成功后更新
 * @param command 输出命令，失败时清零
 * @return 转换成功返回 true，参数或输入状态无效时返回 false
 */
bool RemotePhone_Convert(const srm_control_state_t *raw,
                         RemotePhoneArm_t *arm,
                         RemoteCommand_t *command);

/**
 * @brief 输入一个串口字节，收到完整有效 CONTROL 帧时更新遥控状态
 * @return 成功更新命令返回 true，数据未收齐或无效时返回 false
 */
bool RemotePhone_ProcessByte(srm_parser_t *parser,
                             RemoteInput_t *input,
                             RemotePhoneArm_t *arm,
                             uint8_t byte,
                             uint32_t now_ms);

#endif //XIAOSAI_REMOTE_PHONE_H
