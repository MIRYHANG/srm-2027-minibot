//
// Created by YZH on 2026/10/20.
//

#ifndef XIAOSAI_BMI088_H
#define XIAOSAI_BMI088_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

typedef enum
{
    BMI088_OK = 0,
    BMI088_ERR_ARG,
    BMI088_ERR_SPI,
    BMI088_ERR_ACC_ID,
    BMI088_ERR_GYRO_ID,
    BMI088_ERR_ACC_CONFIG,
    BMI088_ERR_GYRO_CONFIG
} Bmi088Status_t;

typedef enum
{
    /*------写入ACC_RANGE寄存器的值------*/
    BMI088_ACC_RANGE_3G = 0x00,
    BMI088_ACC_RANGE_6G = 0x01,
    BMI088_ACC_RANGE_12G = 0x02,
    BMI088_ACC_RANGE_24G = 0x03
} Bmi088AccRange_t;

typedef enum
{
    /*------写入GYRO_RANGE寄存器的值------*/
    BMI088_GYRO_RANGE_2000DPS = 0x00,
    BMI088_GYRO_RANGE_1000DPS = 0x01,
    BMI088_GYRO_RANGE_500DPS = 0x02,
    BMI088_GYRO_RANGE_250DPS = 0x03,
    BMI088_GYRO_RANGE_125DPS = 0x04
} Bmi088GyroRange_t;

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *acc_cs_port;
    uint16_t acc_cs_pin;
    GPIO_TypeDef *gyro_cs_port;
    uint16_t gyro_cs_pin;
    void (*delay_ms)(uint32_t ms);
    Bmi088AccRange_t acc_range;
    Bmi088GyroRange_t gyro_range;
} Bmi088Config_t;

typedef struct
{
    Bmi088Config_t cfg;
    float acc_scale;   // 原始值 -> m/s^2
    float gyro_scale;  // 原始值 -> rad/s
    bool ready;
} Bmi088_t;

typedef struct
{
    float acc[3];   // m/s^2，芯片坐标系
    float gyro[3];  // rad/s，芯片坐标系，未减零偏
} Bmi088Sample_t;

/**
 * @brief 软复位并配置加速度计和陀螺仪，读回校验每个配置寄存器
 * @note 内部最多延时约 50 ms，调用方需持有 SPI 总线
 * @note 失败时 dev->ready 为 false，可稍后重试
 */
Bmi088Status_t Bmi088_Init(Bmi088_t *dev, const Bmi088Config_t *cfg);

/**
 * @brief 读取一组加速度和角速度
 * @note 不延时，调用方需持有 SPI 总线
 */
Bmi088Status_t Bmi088_ReadSample(const Bmi088_t *dev, Bmi088Sample_t *out);

/**
 * @brief 读取芯片温度，传感器每 1.28 s 更新一次
 */
Bmi088Status_t Bmi088_ReadTemperature(const Bmi088_t *dev, float *temp_c);

#endif //XIAOSAI_BMI088_H
