//
// Created by YZH on 2026/10/20.
//

#include "spi1_bus.h"
#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t spi1_mutex = NULL;

bool Spi1Bus_Init(void)
{
    if (spi1_mutex == NULL)
    {
        // 互斥锁带优先级继承，低优先级任务持锁时不会被中间优先级任务长期阻塞
        spi1_mutex = xSemaphoreCreateMutex();
    }
    return spi1_mutex != NULL;
}

bool Spi1Bus_Lock(uint32_t timeout_ms)
{
    if (spi1_mutex == NULL)
    {
        return false;
    }
    return xSemaphoreTake(spi1_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void Spi1Bus_Unlock(void)
{
    if (spi1_mutex != NULL)
    {
        (void)xSemaphoreGive(spi1_mutex);
    }
}
