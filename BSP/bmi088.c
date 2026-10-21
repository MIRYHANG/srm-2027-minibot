//
// Created by YZH on 2026/10/20.
//

#include "bmi088.h"
#include <stddef.h>
#include <string.h>

#define BMI088_SPI_TIMEOUT_MS       2U
#define BMI088_READ_BIT             0x80U
#define BMI088_MAX_READ_LEN         6U
#define BMI088_SOFTRESET_CMD        0xB6U

/*-----------加速度计寄存器------------*/
#define ACC_CHIP_ID_REG   0x00U
#define ACC_CHIP_ID_VAL   0x1EU
#define ACC_DATA_REG      0x12U  // ACC_X_LSB 起连续 6 字节
#define ACC_TEMP_MSB_REG  0x22U  // TEMP_MSB、TEMP_LSB 连续 2 字节
#define ACC_CONF_REG      0x40U
#define ACC_RANGE_REG     0x41U
#define ACC_PWR_CONF_REG  0x7CU
#define ACC_PWR_CTRL_REG  0x7DU
#define ACC_SOFTRESET_REG 0x7EU

#define ACC_CONF_NORMAL_800HZ 0xABU  // 高 4 位 bwp = normal，低 4 位 odr = 800 Hz
#define ACC_PWR_CONF_ACTIVE   0x00U
#define ACC_PWR_CTRL_ON       0x04U
#define ACC_RANGE_MASK        0x03U

/*-----------陀螺仪寄存器------------*/
#define GYRO_CHIP_ID_REG   0x00U
#define GYRO_CHIP_ID_VAL   0x0FU
#define GYRO_DATA_REG      0x02U  // RATE_X_LSB 起连续 6 字节
#define GYRO_RANGE_REG     0x0FU
#define GYRO_BANDWIDTH_REG 0x10U
#define GYRO_LPM1_REG      0x11U
#define GYRO_SOFTRESET_REG 0x14U

#define GYRO_BW_1000HZ_116HZ 0x02U  // ODR 1000 Hz，滤波带宽 116 Hz
#define GYRO_BW_MASK         0x0FU  // bit7 只读且恒为 1，比较时去掉
#define GYRO_RANGE_MASK      0x07U
#define GYRO_LPM1_NORMAL     0x00U

/*-----------延时，均按数据手册取值并留余量------------*/
#define ACC_RESET_DELAY_MS  2U   // 软复位后 1 ms
#define ACC_POWER_DELAY_MS  5U   // 打开加速度计后
#define GYRO_RESET_DELAY_MS 35U  // 软复位后 30 ms
#define REG_WRITE_DELAY_MS  1U   // 挂起模式下两次写之间至少 450 us

#define REG_MASK_ALL 0xFFU

// 加速度计的 SPI 读在地址后多一个无效字节，陀螺仪没有
#define ACC_DUMMY_BYTES  1U
#define GYRO_DUMMY_BYTES 0U

#define GRAVITY_MPS2   9.80665f
#define DEG_TO_RAD     0.017453292519943f
#define RAW_FULL_SCALE 32768.0f

#define TEMP_RAW_SIGN_LIMIT 1023
#define TEMP_RAW_WRAP       2048
#define TEMP_LSB_DEG        0.125f
#define TEMP_OFFSET_DEG     23.0f

// 下标与 Bmi088AccRange_t / Bmi088GyroRange_t 的取值一一对应
static const float kAccRangeG[] = {3.0f, 6.0f, 12.0f, 24.0f};
static const float kGyroRangeDps[] = {2000.0f, 1000.0f, 500.0f, 250.0f, 125.0f};

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t dummy_bytes;
} Bmi088Chip_t;

typedef struct
{
    uint8_t reg;
    uint8_t value;
    uint8_t mask;      // 读回比较时只看这些位
    uint8_t delay_ms;  // 写入后额外等待
} Bmi088RegSetting_t;

static Bmi088Chip_t AccChip(const Bmi088_t *dev)
{
    return (Bmi088Chip_t){dev->cfg.acc_cs_port, dev->cfg.acc_cs_pin, ACC_DUMMY_BYTES};
}

static Bmi088Chip_t GyroChip(const Bmi088_t *dev)
{
    return (Bmi088Chip_t){dev->cfg.gyro_cs_port, dev->cfg.gyro_cs_pin, GYRO_DUMMY_BYTES};
}

static bool ReadRegs(const Bmi088_t *dev, Bmi088Chip_t chip, uint8_t reg,
                     uint8_t *data, uint16_t len)
{
    uint8_t tx[1U + ACC_DUMMY_BYTES + BMI088_MAX_READ_LEN] = {0};
    uint8_t rx[1U + ACC_DUMMY_BYTES + BMI088_MAX_READ_LEN] = {0};

    if (data == NULL || len == 0U || len > BMI088_MAX_READ_LEN)
    {
        return false;
    }

    uint16_t total = (uint16_t)(1U + chip.dummy_bytes + len);
    tx[0] = (uint8_t)(reg | BMI088_READ_BIT);

    // 一次传输完成地址、无效字节和数据，保证 CS 期间时序连续
    HAL_GPIO_WritePin(chip.port, chip.pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(dev->cfg.hspi, tx, rx, total,
                                                       BMI088_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(chip.port, chip.pin, GPIO_PIN_SET);

    if (status != HAL_OK)
    {
        return false;
    }

    memcpy(data, &rx[1U + chip.dummy_bytes], len);
    return true;
}

static bool WriteReg(const Bmi088_t *dev, Bmi088Chip_t chip, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(reg & (uint8_t)~BMI088_READ_BIT), value};

    HAL_GPIO_WritePin(chip.port, chip.pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->cfg.hspi, tx, 2U, BMI088_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(chip.port, chip.pin, GPIO_PIN_SET);

    return status == HAL_OK;
}

/**
 * @brief 依次写入配置并读回校验
 * @return 全部写入且读回一致返回 true
 */
static bool ApplySettings(const Bmi088_t *dev, Bmi088Chip_t chip,
                          const Bmi088RegSetting_t *settings, size_t count)
{
    for (size_t idx = 0U; idx < count; idx++)
    {
        const Bmi088RegSetting_t *s = &settings[idx];
        uint8_t readback = 0U;

        if (!WriteReg(dev, chip, s->reg, s->value))
        {
            return false;
        }
        dev->cfg.delay_ms((uint32_t)REG_WRITE_DELAY_MS + s->delay_ms);

        if (!ReadRegs(dev, chip, s->reg, &readback, 1U) ||
            (readback & s->mask) != (s->value & s->mask))
        {
            return false;
        }
    }
    return true;
}

static Bmi088Status_t AccInit(const Bmi088_t *dev)
{
    const Bmi088Chip_t chip = AccChip(dev);
    uint8_t id = 0U;

    // 上电后加速度计处于 I2C 模式，CS 出现一次上升沿才切到 SPI，所以先空读一次
    (void)ReadRegs(dev, chip, ACC_CHIP_ID_REG, &id, 1U);
    dev->cfg.delay_ms(REG_WRITE_DELAY_MS);

    if (!WriteReg(dev, chip, ACC_SOFTRESET_REG, BMI088_SOFTRESET_CMD))
    {
        return BMI088_ERR_SPI;
    }
    dev->cfg.delay_ms(ACC_RESET_DELAY_MS);

    // 软复位后又回到 I2C 模式，再空读一次
    (void)ReadRegs(dev, chip, ACC_CHIP_ID_REG, &id, 1U);
    dev->cfg.delay_ms(REG_WRITE_DELAY_MS);

    if (!ReadRegs(dev, chip, ACC_CHIP_ID_REG, &id, 1U))
    {
        return BMI088_ERR_SPI;
    }
    if (id != ACC_CHIP_ID_VAL)
    {
        return BMI088_ERR_ACC_ID;
    }

    // 复位后加速度计处于挂起状态，先切到 active 再打开电源
    const Bmi088RegSetting_t settings[] = {
        {ACC_PWR_CONF_REG, ACC_PWR_CONF_ACTIVE, REG_MASK_ALL, 0U},
        {ACC_PWR_CTRL_REG, ACC_PWR_CTRL_ON, REG_MASK_ALL, ACC_POWER_DELAY_MS},
        {ACC_CONF_REG, ACC_CONF_NORMAL_800HZ, REG_MASK_ALL, 0U},
        {ACC_RANGE_REG, (uint8_t)dev->cfg.acc_range, ACC_RANGE_MASK, 0U},
    };

    return ApplySettings(dev, chip, settings, ARRAY_LEN(settings))
               ? BMI088_OK
               : BMI088_ERR_ACC_CONFIG;
}

static Bmi088Status_t GyroInit(const Bmi088_t *dev)
{
    const Bmi088Chip_t chip = GyroChip(dev);
    uint8_t id = 0U;

    if (!WriteReg(dev, chip, GYRO_SOFTRESET_REG, BMI088_SOFTRESET_CMD))
    {
        return BMI088_ERR_SPI;
    }
    dev->cfg.delay_ms(GYRO_RESET_DELAY_MS);

    if (!ReadRegs(dev, chip, GYRO_CHIP_ID_REG, &id, 1U))
    {
        return BMI088_ERR_SPI;
    }
    if (id != GYRO_CHIP_ID_VAL)
    {
        return BMI088_ERR_GYRO_ID;
    }

    const Bmi088RegSetting_t settings[] = {
        {GYRO_RANGE_REG, (uint8_t)dev->cfg.gyro_range, GYRO_RANGE_MASK, 0U},
        {GYRO_BANDWIDTH_REG, GYRO_BW_1000HZ_116HZ, GYRO_BW_MASK, 0U},
        {GYRO_LPM1_REG, GYRO_LPM1_NORMAL, REG_MASK_ALL, 0U},
    };

    return ApplySettings(dev, chip, settings, ARRAY_LEN(settings))
               ? BMI088_OK
               : BMI088_ERR_GYRO_CONFIG;
}

static bool ConfigValid(const Bmi088Config_t *cfg)
{
    return cfg != NULL && cfg->hspi != NULL && cfg->delay_ms != NULL &&
           cfg->acc_cs_port != NULL && cfg->gyro_cs_port != NULL &&
           (uint32_t)cfg->acc_range < ARRAY_LEN(kAccRangeG) &&
           (uint32_t)cfg->gyro_range < ARRAY_LEN(kGyroRangeDps);
}

Bmi088Status_t Bmi088_Init(Bmi088_t *dev, const Bmi088Config_t *cfg)
{
    if (dev == NULL || !ConfigValid(cfg))
    {
        return BMI088_ERR_ARG;
    }

    // 先在局部变量里完成配置，成功后再整体写回，失败时 dev 保持未就绪
    Bmi088_t next = {
        .cfg = *cfg,
        .acc_scale = kAccRangeG[cfg->acc_range] * GRAVITY_MPS2 / RAW_FULL_SCALE,
        .gyro_scale = kGyroRangeDps[cfg->gyro_range] * DEG_TO_RAD / RAW_FULL_SCALE,
        .ready = false
    };
    dev->ready = false;

    Bmi088Status_t status = AccInit(&next);
    if (status == BMI088_OK)
    {
        status = GyroInit(&next);
    }
    if (status != BMI088_OK)
    {
        return status;
    }

    next.ready = true;
    *dev = next;
    return BMI088_OK;
}

static int16_t ToInt16(const uint8_t *lsb_first)
{
    return (int16_t)(uint16_t)(((uint16_t)lsb_first[1] << 8) | lsb_first[0]);
}

Bmi088Status_t Bmi088_ReadSample(const Bmi088_t *dev, Bmi088Sample_t *out)
{
    uint8_t acc_raw[BMI088_MAX_READ_LEN];
    uint8_t gyro_raw[BMI088_MAX_READ_LEN];

    if (dev == NULL || out == NULL || !dev->ready)
    {
        return BMI088_ERR_ARG;
    }

    if (!ReadRegs(dev, AccChip(dev), ACC_DATA_REG, acc_raw, BMI088_MAX_READ_LEN) ||
        !ReadRegs(dev, GyroChip(dev), GYRO_DATA_REG, gyro_raw, BMI088_MAX_READ_LEN))
    {
        return BMI088_ERR_SPI;
    }

    Bmi088Sample_t sample;
    for (uint32_t axis = 0U; axis < 3U; axis++)
    {
        sample.acc[axis] = (float)ToInt16(&acc_raw[axis * 2U]) * dev->acc_scale;
        sample.gyro[axis] = (float)ToInt16(&gyro_raw[axis * 2U]) * dev->gyro_scale;
    }

    *out = sample;
    return BMI088_OK;
}

Bmi088Status_t Bmi088_ReadTemperature(const Bmi088_t *dev, float *temp_c)
{
    uint8_t raw[2];

    if (dev == NULL || temp_c == NULL || !dev->ready)
    {
        return BMI088_ERR_ARG;
    }
    if (!ReadRegs(dev, AccChip(dev), ACC_TEMP_MSB_REG, raw, 2U))
    {
        return BMI088_ERR_SPI;
    }

    // 11 位补码：MSB 是高 8 位，LSB 的高 3 位是低 3 位
    int32_t value = (int32_t)(((uint32_t)raw[0] << 3) | ((uint32_t)raw[1] >> 5));
    if (value > TEMP_RAW_SIGN_LIMIT)
    {
        value -= TEMP_RAW_WRAP;
    }

    *temp_c = (float)value * TEMP_LSB_DEG + TEMP_OFFSET_DEG;
    return BMI088_OK;
}
