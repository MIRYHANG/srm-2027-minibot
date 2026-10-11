//
// Created by YZH on 2026/10/11.
//

#ifndef XIAOSAI_REMOTE_HANDHELD_H
#define XIAOSAI_REMOTE_HANDHELD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "handheld_packet.h"
#include "remote_input.h"

/**
 * @brief 将已解码的手柄控制包转换为统一遥控命令
 * @param packet 已通过 HhPacket_Decode 校验的控制包
 * @param command 输出命令，失败时清零
 * @return 转换成功返回 true，参数无效返回 false
 * @note 换挡只透传按键状态，按下沿和挡位由 RobotCmd 处理
 */
bool RemoteHandheld_Convert(const HhPacket_t *packet, RemoteCommand_t *command);

/**
 * @brief 输入一包 NRF 载荷，校验通过时更新手柄遥控状态
 * @param now_ms 收到这一包的时间
 * @return 成功更新命令返回 true；载荷无效或参数无效返回 false，原状态不变
 */
bool RemoteHandheld_ProcessPayload(RemoteInput_t *input, const uint8_t *payload,
                                   size_t length, uint32_t now_ms);

#endif //XIAOSAI_REMOTE_HANDHELD_H
