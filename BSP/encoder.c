//
// Created by YZH on 2026/9/25.
//

#include "encoder.h"

#include <stddef.h>
#include <string.h>

void Encoder_Init(Encoder_t *encoder,
                  TIM_HandleTypeDef *htim,
                  float count_per_rev,
                  int8_t direction_sign,
                  float speed_filter_rc_s)
{
    if (encoder == NULL)
    {
        return;
    }

    memset(encoder, 0, sizeof(*encoder));

    if (htim == NULL || count_per_rev <= 0.0f)
    {
        return;
    }

    encoder->htim = htim;
    encoder->count_per_rev = count_per_rev;
    encoder->direction_sign = direction_sign < 0 ? -1 : 1;
    encoder->speed_filter_rc_s = speed_filter_rc_s > 0.0f
                                     ? speed_filter_rc_s
                                     : 0.0f;
}

HAL_StatusTypeDef Encoder_Start(Encoder_t *encoder)
{
    HAL_StatusTypeDef status;

    if (encoder == NULL || encoder->htim == NULL)
    {
        return HAL_ERROR;
    }

    status = HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);
    if (status == HAL_OK)
    {
        encoder->last_count = __HAL_TIM_GET_COUNTER(encoder->htim);
    }

    return status;
}

void Encoder_Reset(Encoder_t *encoder)
{
    if (encoder == NULL || encoder->htim == NULL)
    {
        return;
    }

    __HAL_TIM_SET_COUNTER(encoder->htim, 0U);
    encoder->last_count = 0U;
    encoder->total_count = 0;
    encoder->speed_rpm = 0.0f;
}

void Encoder_Update(Encoder_t *encoder, float dt_s)
{
    uint32_t now_count;
    int32_t delta_count;
    int64_t signed_delta;
    float raw_speed;
    float alpha;

    if (encoder == NULL ||
        encoder->htim == NULL ||
        encoder->count_per_rev <= 0.0f ||
        dt_s <= 0.0f)
    {
        return;
    }

    now_count = __HAL_TIM_GET_COUNTER(encoder->htim);

    // TIM3/TIM4 是 16 位，TIM2/TIM5 是 32 位；减法按对应位宽自然处理回绕。
    if (__HAL_TIM_GET_AUTORELOAD(encoder->htim) <= 0xFFFFU)
    {
        delta_count = (int32_t)(int16_t)((uint16_t)now_count -
                                         (uint16_t)encoder->last_count);
    }
    else
    {
        delta_count = (int32_t)(now_count - encoder->last_count);
    }

    encoder->last_count = now_count;

    signed_delta = (int64_t)delta_count * encoder->direction_sign;
    encoder->total_count += signed_delta;

    raw_speed = ((float)signed_delta * 60.0f) /
                (encoder->count_per_rev * dt_s);

    if (encoder->speed_filter_rc_s <= 0.0f)
    {
        alpha = 1.0f;
    }
    else
    {
        alpha = dt_s / (encoder->speed_filter_rc_s + dt_s);
    }

    encoder->speed_rpm += alpha * (raw_speed - encoder->speed_rpm);
}

int64_t Encoder_GetCount(const Encoder_t *encoder)
{
    return encoder == NULL ? 0 : encoder->total_count;
}

float Encoder_GetAngleDeg(const Encoder_t *encoder)
{
    if (encoder == NULL || encoder->count_per_rev <= 0.0f)
    {
        return 0.0f;
    }

    return (float)encoder->total_count /
           encoder->count_per_rev * 360.0f;
}

float Encoder_GetSpeedRpm(const Encoder_t *encoder)
{
    return encoder == NULL ? 0.0f : encoder->speed_rpm;
}
