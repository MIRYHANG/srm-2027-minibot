//
// Created by YZH on 2026/10/6.
//

#include "oled_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "i2c.h"
#include "i2c_bus_hal.h"
#include "iwdg.h"
#include "main.h"
#include "oled_screen.h"
#include "ssd1306.h"

// 上电后等屏幕电源稳定再初始化，见 Ssd1306_Init 的说明
#define OLED_POWER_UP_MS 100U

// 主循环步长；IWDG 超时约 100 ms，每一步都喂狗
#define LOOP_STEP_MS 10U

// 找不到屏幕时重新初始化的间隔
#define RETRY_PERIOD_MS 500U

// LED_RUN 翻转间隔：屏幕正常时慢闪，找不到屏幕时快闪
#define LED_SLOW_MS 500U
#define LED_FAST_MS 100U

#define MS_PER_S 1000U

// 先试 SA0 接地的 0x3C，再试 0x3D
static const uint8_t OLED_ADDRS[] = {SSD1306_ADDR_LOW, SSD1306_ADDR_HIGH};

static I2cBus_t oled_bus;
static Ssd1306_t oled; // 含 1 KB 显存，必须静态分配

/**
 * @brief 边等边喂狗
 */
static void DelayFeeding(uint32_t ms)
{
    uint32_t start_ms = HAL_GetTick();

    while (HAL_GetTick() - start_ms < ms)
    {
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(1U);
    }
}

/**
 * @brief 依次用每个地址初始化屏幕，成功一个就停
 * @note 总线卡死时每次尝试最多阻塞约 25 ms（HAL 等待总线空闲），每次之前先喂狗
 */
static bool TryInit(void)
{
    for (size_t idx = 0U; idx < sizeof(OLED_ADDRS); idx++)
    {
        HAL_IWDG_Refresh(&hiwdg);
        if (Ssd1306_Init(&oled, &oled_bus, OLED_ADDRS[idx]))
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief 重画整屏并发送有变化的页
 * @note Ssd1306_WriteLine 内容不变时不标记刷新，所以平时每秒只发送运行时间这一页
 * @note 初始化后第一次要发送全部 8 页，I2C2 为 400 kHz 时约 28 ms，在 IWDG 的 100 ms 以内
 * @note 发送失败时 oled.ready 变为 false，主循环会重新初始化
 */
static void DrawScreen(uint32_t uptime_s)
{
    char line[OLED_SCREEN_LINE_SIZE];

    for (uint8_t page = 0U; page < SSD1306_PAGES; page++)
    {
        if (OledScreen_BuildLine(page, oled.addr, uptime_s, line))
        {
            (void)Ssd1306_WriteLine(&oled, page, line);
        }
    }

    HAL_IWDG_Refresh(&hiwdg);
    (void)Ssd1306_Flush(&oled);
}

/**
 * @brief 按屏幕状态闪烁 LED_RUN，屏幕不亮时用来区分程序没跑和屏幕没接好
 */
static void BlinkLed(uint32_t now_ms, uint32_t *last_toggle_ms)
{
    uint32_t period_ms = oled.ready ? LED_SLOW_MS : LED_FAST_MS;

    if (now_ms - *last_toggle_ms >= period_ms)
    {
        *last_toggle_ms = now_ms;
        HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
    }
}

void OledTest_Run(void)
{
    (void)I2cBusHal_Init(&oled_bus, &hi2c2);
    DelayFeeding(OLED_POWER_UP_MS);

    // 第一次进循环立即尝试初始化
    uint32_t last_try_ms = HAL_GetTick() - RETRY_PERIOD_MS;
    uint32_t last_toggle_ms = HAL_GetTick();

    for (;;)
    {
        HAL_IWDG_Refresh(&hiwdg);
        uint32_t now_ms = HAL_GetTick();

        if (!oled.ready && now_ms - last_try_ms >= RETRY_PERIOD_MS)
        {
            last_try_ms = now_ms;
            (void)TryInit();
        }

        if (oled.ready)
        {
            DrawScreen(now_ms / MS_PER_S);
        }

        BlinkLed(now_ms, &last_toggle_ms);
        HAL_Delay(LOOP_STEP_MS);
    }
}
