//
// Created by YZH on 2026/10/22.
//

#include "hh_app.h"
#include "main.h"
#include "adc.h"
#include "hh_input.h"
#include "nrf_link.h"
#include "NRF24.h"
#ifdef HAL_IWDG_MODULE_ENABLED
   #include "iwdg.h"
#endif

#define SAMPLE_MS          5U
#define SEND_MS            10U
#define CAL_SAMPLES        32U
#define STICK_X_INVERT     false
#define STICK_Y_INVERT     false
#define LED_ACTIVE         GPIO_PIN_SET

#define INPUT_TIMEOUT_MS   20U   // 超过这么久没读到摇杆，视为输入异常
#define RADIO_POWERUP_MS   100U  // NRF 模块上电后需要稳定的时间
#define RADIO_RETRY_MS     100U  // 配置失败后的重试间隔
#define RADIO_SETTLE_MS    2U    // 配置完成后等这么久再发第一包
#define LINK_TIMEOUT_MS    100U  // 超过这么久没收到有效 ACK，视为链路断开
#define LED_FAST_HALF_MS   100U  // 快闪：亮 100 ms，灭 100 ms
#define LED_SLOW_HALF_MS   500U  // 慢闪：亮 500 ms，灭 500 ms


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

volatile uint32_t tx_ok_count;
volatile uint32_t tx_lost_count;
volatile uint32_t tx_timeout_count;

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

static bool InputHealthy(uint32_t now)
{
    /*---------------综合消抖、超时、标定后的返回---------------*/
    return calibrated && input_ok && (uint32_t)(now - last_input_ms) <= INPUT_TIMEOUT_MS;
}

static void SendControl(void)
{
    HhInput_t input;
    HhPacket_t packet;

    /*-------------组装输入------------*/
    HhInput_Build(debounce.stable, forward, left, &input);

    if (!InputHealthy(HAL_GetTick()))
    {
        input.estop = true;
    }

    /*---------------把输入指令改成控制指令----------------*/
    if (!HhLogic_Step(&logic, &input, &packet))
    {
        return;
    }

    if (!radio_ready || (uint32_t)(HAL_GetTick() - radio_ready_ms) < RADIO_SETTLE_MS)
    {
        return;
    }

    uint8_t tx[HH_PACKET_LEN];
    uint8_t ack[NRF_LINK_MAX_PAYLOAD];
    uint8_t ack_len = 0U;

    if (HhPacket_Encode(&packet, tx, sizeof(tx)) != HH_PACKET_LEN)
    {
        return;
    }

    NrfTxResult_t result =
        NrfLink_Send(tx, sizeof(tx), ack, &ack_len);

    uint32_t now = HAL_GetTick();

    switch (result)
    {
        case NRF_TX_OK:
            tx_ok_count++;
            break;
        case NRF_TX_LOST:
            tx_lost_count++;
            break;
        default:
            tx_timeout_count++;
            break;
    }

    if (NRF24_Port_HasFault() || result == NRF_TX_TIMEOUT)
    {
        radio_ready = false;
        have_status = false;
        retry_ms = now;
        return;
    }

    if (result == NRF_TX_OK &&
        HhStatus_Decode(ack, ack_len, &car_status) &&
        car_status.gear >= 1U && car_status.gear <= 3U)
    {
        have_status = true;
        status_ms = now;
    }
}

static void WriteLed(GPIO_TypeDef *port, uint16_t pin, bool on)
{
    HAL_GPIO_WritePin(port, pin,on ? LED_ACTIVE : (LED_ACTIVE == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET));
}

/**
 * @brief NRF 未就绪时，上电满 100 ms 后每 100 ms 重试一次配置
 */
static void RadioService(uint32_t now)
{
    if (radio_ready || now < RADIO_POWERUP_MS || (uint32_t)(now - retry_ms) < RADIO_RETRY_MS)
    {
        return;
    }

    retry_ms = now;
    radio_ready = NrfLink_Configure(nRF24_MODE_TX);

    if (radio_ready)
    {
        radio_ready_ms = HAL_GetTick();
    }
}

/**
 * @brief 周期到了返回 true，并推进下一次的时间
 * @note 主循环被耽误超过两个周期时直接对齐到现在，不连续补发好几包
 */
static bool PeriodDue(uint32_t *last_ms, uint32_t period_ms, uint32_t now)
{
    uint32_t elapsed = (uint32_t)(now - *last_ms);

    if (elapsed < period_ms)
    {
        return false;
    }

    *last_ms = (elapsed >= 2U * period_ms) ? now : (*last_ms + period_ms);
    return true;
}

static bool Blink(uint32_t now, uint32_t half_ms)
{
    return ((now / half_ms) % 2U) == 0U;
}

static bool LinkOk(uint32_t now)
{
    return have_status && (uint32_t)(now - status_ms) <= LINK_TIMEOUT_MS;
}

/**
 * @brief LED_STATUS1 显示小车状态
 * @note 链路断开：灭；小车急停或驱动故障：快闪；小车已使能：常亮；小车未使能：慢闪
 */
static bool Led1On(uint32_t now)
{
    if (!LinkOk(now))
    {
        return false;
    }
    if ((car_status.flags & HH_STATUS_SAFETY) != 0U)
    {
        return Blink(now, LED_FAST_HALF_MS);
    }
    if ((car_status.flags & HH_STATUS_ENABLED) != 0U)
    {
        return true;
    }
    return Blink(now, LED_SLOW_HALF_MS);
}

static void UpdateLeds(uint32_t now)
{
    if (!InputHealthy(now))
    {
        // 摇杆未标定（上电时被推着）或 ADC 异常：两灯同步快闪，此时手柄强制失能
        bool on = Blink(now, LED_FAST_HALF_MS);
        WriteLed(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, on);
        WriteLed(LED_STATUS2_GPIO_Port, LED_STATUS2_Pin, on);
        return;
    }

    WriteLed(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, Led1On(now));
    WriteLed(LED_STATUS2_GPIO_Port, LED_STATUS2_Pin, HhLogic_DirLedOn(&logic, now));
}

static void FeedWatchdog(void)
{
#ifdef HAL_IWDG_MODULE_ENABLED
    (void)HAL_IWDG_Refresh(&hiwdg);
#endif
}

void HhApp_Init(void)
{
    HhLogic_Init(&logic);
    HhAxis_Init(&axis_x, STICK_X_INVERT);
    HhAxis_Init(&axis_y, STICK_Y_INVERT);

    // 用第一次采样初始化，上电时就按着的键视为按着，不会被当成一次按下
    HhDebounce_Init(&debounce, ReadButtons());

    // F1 的 ADC 在第一次转换前要做一次自校准；失败时 ReadAxes 会一直报错，手柄保持失能
    (void)HAL_ADCEx_Calibration_Start(&hadc1);

    uint32_t now = HAL_GetTick();
    sample_ms = now;
    send_ms = now;
    retry_ms = now - RADIO_RETRY_MS; // 上电满 RADIO_POWERUP_MS 后立刻尝试第一次配置

    WriteLed(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, false);
    WriteLed(LED_STATUS2_GPIO_Port, LED_STATUS2_Pin, false);
}

void HhApp_Update(void)
{
    if (PeriodDue(&sample_ms, SAMPLE_MS, HAL_GetTick()))
    {
        SampleInput();
    }

    RadioService(HAL_GetTick());

    if (PeriodDue(&send_ms, SEND_MS, HAL_GetTick()))
    {
        SendControl();
    }

    UpdateLeds(HAL_GetTick());
    FeedWatchdog();
}

