//
// Created by YZH on 2026/10/5.
//
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ina226.h"

#define SHUNT_OHM 0.002f
#define TOLERANCE 0.0001f

/* 假总线：模拟一个挂在 addr 上的 INA226，寄存器按高字节在前收发 */
typedef struct
{
    uint8_t addr;
    uint16_t regs[256];
    bool fail_read;
    bool fail_write;
    bool ignore_config_write; // 模拟写入没生效，回读不一致
    int read_count;
} FakeIna_t;

static bool FakeRead(void *ctx, uint8_t addr7, uint8_t reg,
                     uint8_t *data, uint16_t len)
{
    FakeIna_t *fake = ctx;
    fake->read_count++;

    if (fake->fail_read || addr7 != fake->addr || len != 2U)
    {
        return false;
    }

    data[0] = (uint8_t)(fake->regs[reg] >> 8);
    data[1] = (uint8_t)(fake->regs[reg] & 0xFFU);
    return true;
}

static bool FakeWrite(void *ctx, uint8_t addr7, uint8_t reg,
                      const uint8_t *data, uint16_t len)
{
    FakeIna_t *fake = ctx;

    if (fake->fail_write || addr7 != fake->addr || len != 2U)
    {
        return false;
    }

    if (!fake->ignore_config_write)
    {
        fake->regs[reg] = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    }
    return true;
}

static FakeIna_t MakeFake(uint8_t addr)
{
    FakeIna_t fake;
    memset(&fake, 0, sizeof(fake));
    fake.addr = addr;
    fake.regs[0x00] = 0x4127U; // 上电默认配置
    fake.regs[0xFE] = 0x5449U;
    fake.regs[0xFF] = 0x2260U;
    return fake;
}

static I2cBus_t BusOf(FakeIna_t *fake)
{
    return (I2cBus_t){.read = FakeRead, .write = FakeWrite, .ctx = fake};
}

static void AssertNear(float actual, float expected)
{
    assert(fabsf(actual - expected) < TOLERANCE);
}

/* 非法初始化必须返回 false，且 dev 逐字节不变 */
static void AssertInitRejected(const I2cBus_t *bus, uint8_t addr, float shunt_ohm)
{
    Ina226_t dev;
    Ina226_t saved;
    memset(&dev, 0xA5, sizeof(dev));
    memcpy(&saved, &dev, sizeof(saved));

    assert(!Ina226_Init(&dev, bus, addr, shunt_ohm));
    assert(memcmp(&dev, &saved, sizeof(dev)) == 0);
}

static void TestConversions(void)
{
    AssertNear(Ina226_BusVolts(0U), 0.0f);
    AssertNear(Ina226_BusVolts(9600U), 12.0f);  // 9600 × 1.25 mV
    AssertNear(Ina226_BusVolts(0x7FFFU), 40.95875f);

    // 800 × 2.5 μV = 2 mV，除以 2 mΩ 得 1 A
    AssertNear(Ina226_ShuntAmps(800U, SHUNT_OHM), 1.0f);
    AssertNear(Ina226_ShuntAmps((uint16_t)(65536 - 800), SHUNT_OHM), -1.0f);
    AssertNear(Ina226_ShuntAmps(0U, SHUNT_OHM), 0.0f);

    // 量程两端：±81.92 mV / 2 mΩ ≈ ±40.96 A
    AssertNear(Ina226_ShuntAmps(0x7FFFU, SHUNT_OHM), 40.95875f);
    AssertNear(Ina226_ShuntAmps(0x8000U, SHUNT_OHM), -40.96f);
}

static void TestProbe(void)
{
    FakeIna_t fake = MakeFake(0x40U);
    I2cBus_t bus = BusOf(&fake);

    assert(Ina226_Probe(&bus, 0x40U));
    assert(!Ina226_Probe(&bus, 0x41U));

    // 芯片版本号不同仍然认可
    fake.regs[0xFF] = 0x2261U;
    assert(Ina226_Probe(&bus, 0x40U));

    fake.regs[0xFF] = 0x3220U;
    assert(!Ina226_Probe(&bus, 0x40U));

    fake.regs[0xFF] = 0x2260U;
    fake.regs[0xFE] = 0x0000U;
    assert(!Ina226_Probe(&bus, 0x40U));

    // 地址超出 INA226 范围、总线接口不完整
    fake.regs[0xFE] = 0x5449U;
    assert(!Ina226_Probe(&bus, 0x3CU));
    assert(!Ina226_Probe(&bus, 0x50U));
    assert(!Ina226_Probe(NULL, 0x40U));

    I2cBus_t broken = bus;
    broken.read = NULL;
    assert(!Ina226_Probe(&broken, 0x40U));
}

static void TestFindAddress(void)
{
    // 优先地址上就有芯片：只试一次
    FakeIna_t fake = MakeFake(0x40U);
    I2cBus_t bus = BusOf(&fake);
    uint8_t found = 0U;

    assert(Ina226_FindAddress(&bus, 0x40U, &found));
    assert(found == 0x40U);
    assert(fake.read_count == 2);

    // 芯片在 0x45：扫描后找到
    fake = MakeFake(0x45U);
    bus = BusOf(&fake);
    found = 0U;
    assert(Ina226_FindAddress(&bus, 0x40U, &found));
    assert(found == 0x45U);

    // 总线上没有芯片：返回 false，found 不变；每个地址只读一次
    fake = MakeFake(0x60U);
    bus = BusOf(&fake);
    found = 0x12U;
    assert(!Ina226_FindAddress(&bus, 0x40U, &found));
    assert(found == 0x12U);
    assert(fake.read_count == 16);

    assert(!Ina226_FindAddress(&bus, 0x40U, NULL));
}

static void TestInit(void)
{
    FakeIna_t fake = MakeFake(0x40U);
    I2cBus_t bus = BusOf(&fake);
    Ina226_t dev;

    assert(Ina226_Init(&dev, &bus, 0x40U, SHUNT_OHM));
    assert(dev.ready);
    assert(dev.addr == 0x40U);
    assert(dev.shunt_ohm == SHUNT_OHM);
    assert(fake.regs[0x00] == 0x4527U);

    // 非法参数
    assert(!Ina226_Init(NULL, &bus, 0x40U, SHUNT_OHM));
    AssertInitRejected(NULL, 0x40U, SHUNT_OHM);
    AssertInitRejected(&bus, 0x3CU, SHUNT_OHM);
    AssertInitRejected(&bus, 0x41U, SHUNT_OHM);

    static const float bad_shunt[] = {0.0f, -0.002f, NAN, INFINITY};
    for (size_t idx = 0; idx < sizeof(bad_shunt) / sizeof(bad_shunt[0]); idx++)
    {
        AssertInitRejected(&bus, 0x40U, bad_shunt[idx]);
    }

    // 写配置失败、回读不一致
    fake = MakeFake(0x40U);
    fake.fail_write = true;
    AssertInitRejected(&bus, 0x40U, SHUNT_OHM);

    fake = MakeFake(0x40U);
    fake.ignore_config_write = true;
    AssertInitRejected(&bus, 0x40U, SHUNT_OHM);
}

static void TestRead(void)
{
    FakeIna_t fake = MakeFake(0x40U);
    I2cBus_t bus = BusOf(&fake);
    Ina226_t dev;
    assert(Ina226_Init(&dev, &bus, 0x40U, SHUNT_OHM));

    // 12 V、1.25 A → 15 W
    fake.regs[0x01] = 1000U;
    fake.regs[0x02] = 9600U;
    Ina226Reading_t reading;
    assert(Ina226_Read(&dev, &reading));
    AssertNear(reading.bus_v, 12.0f);
    AssertNear(reading.current_a, 1.25f);
    AssertNear(reading.power_w, 15.0f);

    // 反向电流：功率为负
    fake.regs[0x01] = (uint16_t)(65536 - 1000);
    assert(Ina226_Read(&dev, &reading));
    AssertNear(reading.current_a, -1.25f);
    AssertNear(reading.power_w, -15.0f);

    // 母线电压保留位为 1：数据不可信，out 不变，但不需要重新初始化
    Ina226Reading_t saved = reading;
    fake.regs[0x02] = 0x8000U | 9600U;
    assert(!Ina226_Read(&dev, &reading));
    assert(memcmp(&reading, &saved, sizeof(reading)) == 0);
    assert(dev.ready);

    // 总线失败：out 不变，ready 清零，之后不再读取
    fake.regs[0x02] = 9600U;
    fake.fail_read = true;
    assert(!Ina226_Read(&dev, &reading));
    assert(memcmp(&reading, &saved, sizeof(reading)) == 0);
    assert(!dev.ready);

    fake.fail_read = false;
    assert(!Ina226_Read(&dev, &reading));

    // 重新初始化后恢复
    assert(Ina226_Init(&dev, &bus, 0x40U, SHUNT_OHM));
    assert(Ina226_Read(&dev, &reading));

    assert(!Ina226_Read(NULL, &reading));
    assert(!Ina226_Read(&dev, NULL));
}

int main(void)
{
    TestConversions();
    TestProbe();
    TestFindAddress();
    TestInit();
    TestRead();

    puts("INA226 tests passed");
    return 0;
}
