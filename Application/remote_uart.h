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

/**
 * @brief 启动 USART2 单字节中断接收。
 * @return 启动成功返回 true；未初始化或启动失败返回 false。
 * @note 初始化队列后，在任务启动阶段调用一次。
 */
bool RemoteUart_Start(void);

/**
 * @brief 非阻塞地取出一个接收记录，仅供任务调用。
 * @param item 输出收到的字节和接收时间；返回 false 时不要使用其内容。
 * @return 取出成功返回 true；队列为空、参数无效或接收异常返回 false。
 */
bool RemoteUart_Read(RemoteUartByte_t *item);

#endif //XIAOSAI_REMOTE_UART_H
