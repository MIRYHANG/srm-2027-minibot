//
// Created by YZH on 2026/10/5.
//

#ifndef XIAOSAI_I2C_BUS_HAL_H
#define XIAOSAI_I2C_BUS_HAL_H

#include <stdbool.h>

#include "i2c_bus.h"
#include "main.h"

/**
 * @brief 用 HAL I2C 句柄填充总线接口
 * @return hi2c 和 bus 都非空返回 true
 * @note 每次读写的超时为 I2C_BUS_HAL_TIMEOUT_MS，只能在任务中调用，不能在中断或临界区中调用
 */
bool I2cBusHal_Init(I2cBus_t *bus, I2C_HandleTypeDef *hi2c);

#endif //XIAOSAI_I2C_BUS_HAL_H
