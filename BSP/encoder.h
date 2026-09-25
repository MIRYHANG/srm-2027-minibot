//
// Created by YZH on 2026/9/25.
//

#ifndef XIAOSAI_ENCODER_H
#define XIAOSAI_ENCODER_H

#include <stdint.h>

#include "tim.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t last_count;
    int64_t total_count;
    float count_per_rev;              // 输出轴转一圈对应的定时器计数值
    float speed_rpm;
    float speed_filter_rc_s;          // 转速一阶低通滤波时间常数，单位：s
    int8_t direction_sign;            // 方向修正，只取 +1 或 -1
} Encoder_t;

void Encoder_Init(Encoder_t *encoder,
                  TIM_HandleTypeDef *htim,
                  float count_per_rev,
                  int8_t direction_sign,
                  float speed_filter_rc_s);

HAL_StatusTypeDef Encoder_Start(Encoder_t *encoder);
void Encoder_Reset(Encoder_t *encoder);
void Encoder_Update(Encoder_t *encoder, float dt_s);
int64_t Encoder_GetCount(const Encoder_t *encoder);
float Encoder_GetAngleDeg(const Encoder_t *encoder);
float Encoder_GetSpeedRpm(const Encoder_t *encoder);

#endif // XIAOSAI_ENCODER_H
