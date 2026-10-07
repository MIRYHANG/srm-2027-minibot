//
// Created by YZH on 2026/10/5.
//

#ifndef XIAOSAI_I2C_BUS_H
#define XIAOSAI_I2C_BUS_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief I2C 寄存器读写接口，不依赖 HAL
 * @note 固件里由 i2c_bus_hal.c 实现，主机测试里换成假总线
 * @note addr7 为 7 位地址；reg 为器件内部寄存器地址；成功返回 true
 */
typedef struct
{
    bool (*read)(void *ctx, uint8_t addr7, uint8_t reg,
                 uint8_t *data, uint16_t len);
    bool (*write)(void *ctx, uint8_t addr7, uint8_t reg,
                  const uint8_t *data, uint16_t len);
    void *ctx;
} I2cBus_t;

#endif //XIAOSAI_I2C_BUS_H
