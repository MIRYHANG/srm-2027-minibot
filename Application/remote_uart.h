//
// Created by YZH on 2026/9/28.
//

#ifndef XIAOSAI_REMOTE_UART_H
#define XIAOSAI_REMOTE_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t byte;
    uint32_t received_ms;
} RemoteUartByte_t;

/**
 * @brief 创建串口接收队列，启动接收前调用一次。
 * @return 创建成功返回 true，失败返回 false。
 */
bool RemoteUart_Init(void);
#endif //XIAOSAI_REMOTE_UART_H
