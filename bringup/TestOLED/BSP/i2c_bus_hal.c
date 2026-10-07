//
// Created by YZH on 2026/10/5.
//

#include "i2c_bus_hal.h"

#include <stddef.h>

// 单次传输超时；总线卡死时任务最多等这么久，不能用 HAL_MAX_DELAY
#define I2C_BUS_HAL_TIMEOUT_MS 10U

static bool HalRead(void *ctx, uint8_t addr7, uint8_t reg,
                    uint8_t *data, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = ctx;

    if (hi2c == NULL || data == NULL || len == 0U || addr7 > 0x7FU)
    {
        return false;
    }

    // HAL 使用左移一位后的 8 位地址
    return HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr7 << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, data, len,
                            I2C_BUS_HAL_TIMEOUT_MS) == HAL_OK;
}

static bool HalWrite(void *ctx, uint8_t addr7, uint8_t reg,
                     const uint8_t *data, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = ctx;

    if (hi2c == NULL || data == NULL || len == 0U || addr7 > 0x7FU)
    {
        return false;
    }

    // HAL 的参数不是 const，但写操作不会修改 data
    return HAL_I2C_Mem_Write(hi2c, (uint16_t)(addr7 << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len,
                             I2C_BUS_HAL_TIMEOUT_MS) == HAL_OK;
}

bool I2cBusHal_Init(I2cBus_t *bus, I2C_HandleTypeDef *hi2c)
{
    if (bus == NULL || hi2c == NULL)
    {
        return false;
    }

    *bus = (I2cBus_t){
        .read = HalRead,
        .write = HalWrite,
        .ctx = hi2c,
    };
    return true;
}