//
// Created by YZH on 2026/10/5.
//

#include "ina226.h"

#include <math.h>
#include <stddef.h>

#define INA226_REG_CONFIG  0x00U
#define INA226_REG_SHUNT   0x01U
#define INA226_REG_BUS     0x02U
#define INA226_REG_MANUF   0xFEU
#define INA226_REG_DIE     0xFFU

#define INA226_MANUF_ID    0x5449U // ASCII "TI"
#define INA226_DIE_ID      0x2260U
#define INA226_DIE_ID_MASK 0xFFF0U // 低 4 位是芯片版本号，不参与比较

// 配置寄存器：bit14 固定为 1 | 平均 16 次 (010) | 母线 1.1 ms (100) | 分流 1.1 ms (100) | 连续测量 (111)
#define INA226_CONFIG_VALUE 0x4527U

#define INA226_BUS_LSB_V    0.00125f
#define INA226_SHUNT_LSB_V  0.0000025f

// 母线电压寄存器最高位恒为 0，读到 1 说明数据不可信
#define INA226_BUS_RESERVED_BIT 0x8000U

/**
 * @brief 读一个 16 位寄存器，芯片按高字节在前发送
 */
static bool ReadReg(const I2cBus_t *bus, uint8_t addr, uint8_t reg, uint16_t *value)
{
    uint8_t data[2];

    if (!bus->read(bus->ctx, addr, reg, data, 2U))
    {
        return false;
    }

    *value = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    return true;
}

static bool WriteReg(const I2cBus_t *bus, uint8_t addr, uint8_t reg, uint16_t value)
{
    const uint8_t data[2] = {(uint8_t)(value >> 8), (uint8_t)(value & 0xFFU)};

    return bus->write(bus->ctx, addr, reg, data, 2U);
}

static bool BusValid(const I2cBus_t *bus)
{
    return bus != NULL && bus->read != NULL && bus->write != NULL;
}

static bool AddrValid(uint8_t addr)
{
    return addr >= INA226_ADDR_MIN && addr <= INA226_ADDR_MAX;
}

/**
 * @brief 把 16 位补码转成有符号数，不依赖编译器对强制转换的实现
 */
static int32_t ToSigned16(uint16_t raw)
{
    return (raw & 0x8000U) != 0U ? (int32_t)raw - 65536 : (int32_t)raw;
}

float Ina226_BusVolts(uint16_t raw)
{
    return (float)raw * INA226_BUS_LSB_V;
}

float Ina226_ShuntAmps(uint16_t raw, float shunt_ohm)
{
    return (float)ToSigned16(raw) * INA226_SHUNT_LSB_V / shunt_ohm;
}

bool Ina226_Probe(const I2cBus_t *bus, uint8_t addr)
{
    if (!BusValid(bus) || !AddrValid(addr))
    {
        return false;
    }

    uint16_t manuf = 0U;
    uint16_t die = 0U;

    // 先读厂商 ID：地址上没有器件时第一次读取就会失败，扫描更快
    if (!ReadReg(bus, addr, INA226_REG_MANUF, &manuf) || manuf != INA226_MANUF_ID)
    {
        return false;
    }

    if (!ReadReg(bus, addr, INA226_REG_DIE, &die))
    {
        return false;
    }

    return (die & INA226_DIE_ID_MASK) == INA226_DIE_ID;
}

bool Ina226_FindAddress(const I2cBus_t *bus, uint8_t preferred, uint8_t *found)
{
    if (found == NULL)
    {
        return false;
    }

    if (Ina226_Probe(bus, preferred))
    {
        *found = preferred;
        return true;
    }

    for (uint8_t addr = INA226_ADDR_MIN; addr <= INA226_ADDR_MAX; addr++)
    {
        if (addr != preferred && Ina226_Probe(bus, addr))
        {
            *found = addr;
            return true;
        }
    }

    return false;
}

bool Ina226_Init(Ina226_t *dev, const I2cBus_t *bus, uint8_t addr, float shunt_ohm)
{
    if (dev == NULL || !isfinite(shunt_ohm) || shunt_ohm <= 0.0f)
    {
        return false;
    }

    // Probe 同时检查总线接口和地址
    if (!Ina226_Probe(bus, addr))
    {
        return false;
    }

    uint16_t readback = 0U;

    if (!WriteReg(bus, addr, INA226_REG_CONFIG, INA226_CONFIG_VALUE) ||
        !ReadReg(bus, addr, INA226_REG_CONFIG, &readback) ||
        readback != INA226_CONFIG_VALUE)
    {
        return false;
    }

    *dev = (Ina226_t){
        .bus = *bus,
        .addr = addr,
        .shunt_ohm = shunt_ohm,
        .ready = true,
    };
    return true;
}

bool Ina226_Read(Ina226_t *dev, Ina226Reading_t *out)
{
    if (dev == NULL || out == NULL || !dev->ready)
    {
        return false;
    }

    uint16_t shunt_raw = 0U;
    uint16_t bus_raw = 0U;

    if (!ReadReg(&dev->bus, dev->addr, INA226_REG_SHUNT, &shunt_raw) ||
        !ReadReg(&dev->bus, dev->addr, INA226_REG_BUS, &bus_raw))
    {
        // 芯片可能掉电重启或总线出错，交给调用方重新初始化
        dev->ready = false;
        return false;
    }

    if ((bus_raw & INA226_BUS_RESERVED_BIT) != 0U)
    {
        return false;
    }

    float bus_v = Ina226_BusVolts(bus_raw);
    float current_a = Ina226_ShuntAmps(shunt_raw, dev->shunt_ohm);

    *out = (Ina226Reading_t){
        .bus_v = bus_v,
        .current_a = current_a,
        .power_w = bus_v * current_a,
    };
    return true;
}
