//
// Created by YZH on 2026/10/22.
//

#include "hh_app.h"
#include "main.h"
#include "adc.h"
#include "hh_input.h"
#include "nrf_link.h"
#include "NRF24.h"

#define SAMPLE_MS          5U
#define SEND_MS            10U
#define CAL_SAMPLES        32U
#define STICK_X_INVERT     false
#define STICK_Y_INVERT     false
#define LED_ACTIVE         GPIO_PIN_SET

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} Button_Pin;

#define BUTTON(id, name) [id] = {name##_GPIO_Port, name##_Pin} // 双##表示记号拼接

static const Button_Pin buttons[HH_BTN_COUNT] = {
    BUTTON(HH_BTN_START, REMOTE_START_BTN),
    BUTTON(HH_BTN_ESTOP, REMOTE_ESTOP_BTN),
    BUTTON(HH_BTN_GEAR, JOY_BTN),
    BUTTON(HH_BTN_TURN_CCW, CHASSIS_CCW_BTN),
    BUTTON(HH_BTN_TURN_CW, CHASSIS_CW_BTN),
    BUTTON(HH_BTN_ARM_DIR, ARM_DIR_TOGGLE_BTN),
    BUTTON(HH_BTN_ARM_STOP, ARM_STOP_BTN),
    BUTTON(HH_BTN_J1, ARM_J1_BTN),
    BUTTON(HH_BTN_J2, ARM_J2_BTN),
    BUTTON(HH_BTN_J3, ARM_J3_BTN),
    BUTTON(HH_BTN_J4, ARM_J4_BTN),
    BUTTON(HH_BTN_J5, ARM_J5_BTN),
    BUTTON(HH_BTN_GRIPPER, ARM_GRIPPER_BTN)
};

#undef BUTTON
/*---------------至此删除宏----------------*/

_Alignas(4) static volatile uint16_t adc_dma[2];
static volatile bool adc_done;
static volatile bool adc_error;

static HhDebounce_t debounce;
static HhLogic_t logic;
static HhAxis_t axis_x, axis_y;

static uint16_t cal_x[CAL_SAMPLES], cal_y[CAL_SAMPLES];
static unsigned cal_count;
static bool calibrated, input_ok, radio_ready, have_status;
static int16_t forward, left;
static HhStatus_t car_status;

static uint32_t sample_ms, send_ms, retry_ms, radio_ready_ms;
static uint32_t last_input_ms, status_ms;

static uint16_t ReadButtons(void)
{
    uint16_t value = 0U;

    for (unsigned i = 0U; i < HH_BTN_COUNT; ++i)
    {
        if (HAL_GPIO_ReadPin(buttons[i].port, buttons[i].pin) == GPIO_PIN_RESET)
        {
            /*----------------把第i位的值置为1，表示其按下--------------*/
            value |= HH_BTN_BIT(i);
        }
    }

    return value;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc)
{
    if (adc == &hadc1)
    {
        adc_done = true;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *adc)
{
    if (adc == &hadc1)
    {
        adc_error = true;
    }
}

/*-------------两通道单次扫描，等待3ms------------*/
static bool ReadAxes(uint16_t *x, uint16_t *y)
{
    adc_done = false;
    adc_error = false;

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma, 2U) != HAL_OK)
    {
        (void)HAL_ADC_Stop_DMA(&hadc1);
        return false;
    }

    uint32_t start = HAL_GetTick();

    while (!adc_done && !adc_error && (uint32_t)(HAL_GetTick() - start) < 3U)
    {
        /*----------等待采样结束---------*/
    }

    bool ok = adc_done && !adc_error;

    if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
    {
        ok = false;
    }

    if (!ok)
    {
        return false;
    }

    *x = adc_dma[0]; /* rank 1：PA0 / X */
    *y = adc_dma[1]; /* rank 2：PA1 / Y */

    return *x <= HH_ADC_MAX && *y <= HH_ADC_MAX;
}

static bool StableCenter(const uint16_t *values)
{
    uint16_t lo = values[0], hi = values[0];

    for (unsigned i = 1U; i < CAL_SAMPLES; ++i)
    {
        if (values[i] < lo)
        {
            /*----------更新最小值---------*/
            lo = values[i];
        }
        if (values[i] > hi)
        {
            /*----------更新最大值---------*/
            hi = values[i];
        }
    }

    /*------------死区标定判断--------------*/
    return (uint16_t)(hi - lo) <= HH_STICK_DEADZONE;
}

static void SampleInput(void)
{
    /*-------------------读取按键-------------------*/
    (void)HhDebounce_Update(&debounce, ReadButtons());
    
    uint16_t x, y;
    input_ok = ReadAxes(&x, &y);

    if (!input_ok)
    {
        cal_count = 0U;
        return;
    }

    last_input_ms = HAL_GetTick();

    if (!calibrated)
    {
        cal_x[cal_count] = x;
        cal_y[cal_count++] = y;

        if (cal_count == CAL_SAMPLES)
        {
            /*------------标定中点--------------*/
            calibrated =
                StableCenter(cal_x) &&
                StableCenter(cal_y) &&
                HhAxis_Calibrate(&axis_x, cal_x, CAL_SAMPLES) &&
                HhAxis_Calibrate(&axis_y, cal_y, CAL_SAMPLES);

            cal_count = 0U;
        }
    }

    forward = HhAxis_Normalize(&axis_y, y);
    left = HhAxis_Normalize(&axis_x, x);
}
