//
// Created by YZH on 2026/9/27
//

#ifndef XIAOSAI_REMOTE_PHONE_H
#define XIAOSAI_REMOTE_PHONE_H

#include <stdbool.h>
#include "srm_protocol.h"
#include "remote_input.h"

/**
 * @brief 将已解析的手机遥控状态转换为统一遥控命令
 * @param raw 输入的协议状态，只读；四个摇杆轴均须在 [-512, 511] 范围内
 * @param command 输出命令，保存归一化后的运动量、使能标志和停机请求
 * @return 转换成功返回 true；任一指针为空或摇杆值越界返回 false
 * @note command 非空时，转换失败会将其清零（禁止运动）
 *       本函数不负责接收数据、校验报文、判断掉线或直接控制电机
 */
bool RemotePhone_Convert(const srm_control_state_t *raw,
                         RemoteCommand_t *command);

/**
 * @brief 处理一个手机遥控字节；收到完整有效的控制帧后更新手机状态
 * @return 成功更新命令返回 true；数据未收齐或无效返回 false
 */
bool RemotePhone_ProcessByte(srm_parser_t *parser,
                             RemoteInput_t *input,
                             uint8_t byte,
                             uint32_t now_ms);

#endif //XIAOSAI_REMOTE_PHONE_H
