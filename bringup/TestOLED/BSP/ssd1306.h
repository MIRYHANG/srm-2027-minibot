//
// Created by YZH on 2026/10/5.
//

#ifndef XIAOSAI_SSD1306_H
#define XIAOSAI_SSD1306_H

#include <stdbool.h>
#include <stdint.h>

#include "i2c_bus.h"

#define SSD1306_WIDTH     128U
#define SSD1306_PAGES     8U   // 64 行，每页 8 行
#define SSD1306_TEXT_COLS 21U  // 每个字符占 6 列：5 列字形 + 1 列间隔

// SA0 脚接低电平为 0x3C，接高电平为 0x3D
#define SSD1306_ADDR_LOW  0x3CU
#define SSD1306_ADDR_HIGH 0x3DU

typedef struct
{
    I2cBus_t bus;
    uint8_t addr;
    uint8_t buffer[SSD1306_PAGES][SSD1306_WIDTH]; // 显存副本，1 KB
    uint8_t dirty;                                // 每一位对应一页，1 表示需要发送到屏幕
    bool ready;
} Ssd1306_t;

/**
 * @brief 发送初始化命令，清空显存副本并标记所有页待刷新
 * @return 成功返回 true；失败返回 false，此时 dev->ready 为 false
 * @note 结构体有 1 KB 显存，必须定义为静态变量，不能放在任务栈上
 * @note 模块上电后要等电源稳定（约 100 ms）再调用
 */
bool Ssd1306_Init(Ssd1306_t *dev, const I2cBus_t *bus, uint8_t addr);

/**
 * @brief 把一行文字画进显存副本的某一页，整行重画，多出来的字符截掉
 * @param page 0～7，对应屏幕上第 1～8 行文字
 * @return 参数有效返回 true
 * @note 只改显存副本，内容有变化时才标记该页待刷新；需要调用 Ssd1306_Flush 才会显示
 */
bool Ssd1306_WriteLine(Ssd1306_t *dev, uint8_t page, const char *text);

/**
 * @brief 把待刷新的页发送到屏幕
 * @return 全部发送成功返回 true
 * @note 发送失败时该页保持待刷新，dev->ready 置为 false，需要重新初始化
 */
bool Ssd1306_Flush(Ssd1306_t *dev);

#endif //XIAOSAI_SSD1306_H
