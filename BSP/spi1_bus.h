//
// Created by YZH on 2026/10/20.
//

#ifndef XIAOSAI_SPI1_BUS_H
#define XIAOSAI_SPI1_BUS_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 创建 SPI1 互斥锁，BMI088 与 NRF24 共用 SPI1
 * @note 在 MX_FREERTOS_Init 中、任务创建前调用一次
 */
bool Spi1Bus_Init(void);

/**
 * @brief 获取 SPI1，超时返回 false，此时不得访问 SPI1
 * @note 只在任务中调用，不在中断中调用
 */
bool Spi1Bus_Lock(uint32_t timeout_ms);

/**
 * @brief 释放 SPI1，只能由持有者调用
 */
void Spi1Bus_Unlock(void);

#endif //XIAOSAI_SPI1_BUS_H
