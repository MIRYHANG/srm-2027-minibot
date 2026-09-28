//
// Created by YZH on 2026/9/28.
//

#include "remote_uart.h"
#include "FreeRTOS.h"
#include "queue.h"

#define REMOTE_UART_QUEUE_LENGTH 64

static QueueHandle_t rx_queue = NULL;

bool RemoteUart_Init(void)
{
    // 已经创建过，就不重复创建
    if (rx_queue != NULL)
    {
        return true;
    }

    rx_queue = xQueueCreate(REMOTE_UART_QUEUE_LENGTH,sizeof(RemoteUartByte_t));

    // 非空表示创建成功,为空表示创建失败
    return rx_queue != NULL;
}
