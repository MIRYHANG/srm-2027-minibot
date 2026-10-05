//
// Created by YZH on 2026/10/5.
//

#ifndef XIAOSAI_INA226_H
#define XIAOSAI_INA226_H

#include <stdbool.h>
#include <stdint.h>

#include "i2c_bus.h"

// A0、A1 两个地址脚的 16 种接法对应的 7 位地址范围
#define INA226_ADDR_MIN 0x40U
#define INA226_ADDR_MAX 0x4FU

typedef struct
{
    I2cBus_t bus;
    uint8_t addr;    // 7 位地址
    float shunt_ohm; // 分流电阻阻值
    bool ready;      // 初始化成功且最近一次读取没有总线错误
} Ina226_t;

typedef struct
{
    float bus_v;     // 母线电压，单位 V
    float current_a; // 电流，单位 A；正负取决于分流电阻的接法
    float power_w;   // 功率，单位 W，等于 bus_v × current_a
} Ina226Reading_t;

/**
 * @brief 母线电压寄存器原始值换算成伏特，LSB 为 1.25 mV
 */
float Ina226_BusVolts(uint16_t raw);

/**
 * @brief 分流电压寄存器原始值换算成安培
 * @param raw 有符号 16 位补码，LSB 为 2.5 μV
 * @param shunt_ohm 分流电阻阻值，调用方保证为正
 */
float Ina226_ShuntAmps(uint16_t raw, float shunt_ohm);

/**
 * @brief 读取厂商 ID 和芯片 ID，确认该地址上是 INA226
 * @return 两个 ID 都正确返回 true
 */
bool Ina226_Probe(const I2cBus_t *bus, uint8_t addr);

/**
 * @brief 先试 preferred，再从 0x40 到 0x4F 逐个查找 INA226
 * @return 找到返回 true 并写入 found；找不到返回 false，found 不变
 * @note 总线卡死时每个地址最多等一次 I2C 超时
 */
bool Ina226_FindAddress(const I2cBus_t *bus, uint8_t preferred, uint8_t *found);

/**
 * @brief 确认芯片、写入配置并回读校验
 * @return 成功返回 true 且 dev->ready 为 true；失败返回 false，且不修改 dev
 * @note 配置为 16 次平均、母线和分流各 1.1 ms 转换、连续测量，一组数据约 35 ms 更新一次
 */
bool Ina226_Init(Ina226_t *dev, const I2cBus_t *bus, uint8_t addr, float shunt_ohm);

/**
 * @brief 读取一次电压、电流和功率
 * @return 成功返回 true 并写入 out；失败返回 false，out 不变
 * @note 总线读取失败时 dev->ready 置为 false，需要重新初始化
 */
bool Ina226_Read(Ina226_t *dev, Ina226Reading_t *out);

#endif //XIAOSAI_INA226_H
