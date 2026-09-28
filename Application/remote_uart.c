//
// Created by YZH on 2026/9/28.
//

#include "remote_uart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "usart.h"
#include "task.h"

#define REMOTE_UART_QUEUE_LENGTH 64

static QueueHandle_t rx_queue = NULL;
static uint8_t rx_byte;
static volatile bool rx_fault = false;  // 记录接收异常，后续由任务处理。

bool RemoteUart_Init(void)
{
    // 已经创建过，就不重复创建
    if (rx_queue != NULL)
    {
        return true;
    }

    // 队列可以存 64 个完整结构体，每个都包含 byte 和 received_ms
    rx_queue = xQueueCreate(REMOTE_UART_QUEUE_LENGTH,sizeof(RemoteUartByte_t));

    // 非空表示创建成功,为空表示创建失败
    return rx_queue != NULL;
}

bool RemoteUart_Start(void)
{
    if (rx_queue == NULL)
    {
        return false;
    }

    return HAL_UART_Receive_IT(&huart2, (uint8_t *)(&rx_byte), 1) == HAL_OK;
}

/**
 * @brief 串口接收完成回调，由 HAL 在中断中调用。
 * @note 只暂存字节并继续接收，不解析协议、不控制电机。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2 || rx_queue == NULL)
    {
        return;
    }

    RemoteUartByte_t item = {
        .byte = rx_byte,
        .received_ms = HAL_GetTick()
    };

    // 用来记录：发送到队列后，是否需要唤醒更高优先级任务
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (xQueueSendFromISR(rx_queue, &item,
                     &higher_priority_task_woken) != pdPASS)
    {
        rx_fault = true;
    }

    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1U) != HAL_OK)
    {
        rx_fault = true;
    }

    // 如果刚才唤醒了更高优先级任务，请求在中断退出时切换
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

bool RemoteUart_Read(RemoteUartByte_t *item)
{
    if (rx_queue == NULL || item == NULL || rx_fault)
    {
        return false;
    }

    // 从队列头取出一项，复制到 item 指向的结构体。
    // 等待时间为 0：队列为空就立即返回，不阻塞任务。
    return xQueueReceive(rx_queue, item, 0) == pdPASS;
}

bool RemoteUart_HasFault(void)
{
    return rx_fault;
}

bool RemoteUart_Recover(void)
{
    if (rx_queue == NULL)
    {
        return false;
    }

    HAL_StatusTypeDef status;

    // 暂时阻止 USART2 中断插入，避免恢复过程被打断
    taskENTER_CRITICAL();

    // 停止当前接收；HAL 同时清除接收错误标志和残留数据
    status = HAL_UART_AbortReceive(&huart2);

    if (status == HAL_OK)
    {
        // 已经丢过字节，旧缓存不能再继续拼帧
        xQueueReset(rx_queue);

        rx_fault = false;

        // 重新等待一个字节
        status = HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);

        if (status != HAL_OK)
        {
            rx_fault = true;
        }
    }
    else
    {
        rx_fault = true;
    }

    taskEXIT_CRITICAL();

    return status == HAL_OK;
}

/**
 * @brief HAL 检测到串口通信错误时调用。
 * @note 中断中只标记异常，恢复由任务完成。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        rx_fault = true;
    }
}