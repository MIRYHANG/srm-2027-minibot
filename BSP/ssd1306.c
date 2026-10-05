//
// Created by YZH on 2026/10/5.
//

#include "ssd1306.h"

#include <stddef.h>
#include <string.h>

#include "font5x7.h"

// I2C 传输的第一个字节是控制字节：0x00 后面跟命令，0x40 后面跟显存数据
#define SSD1306_CTRL_CMD  0x00U
#define SSD1306_CTRL_DATA 0x40U

// 每次传输的数据字节数；100 kHz 下约 1.6 ms，远小于 I2C 的 10 ms 超时
#define SSD1306_CHUNK_BYTES 16U

#define SSD1306_ALL_PAGES 0xFFU

// 128×64、内部电荷泵、页寻址模式
static const uint8_t INIT_CMDS[] = {
    0xAE,       // 关显示
    0xD5, 0x80, // 时钟分频
    0xA8, 0x3F, // 复用率 64
    0xD3, 0x00, // 显示偏移 0
    0x40,       // 起始行 0
    0x8D, 0x14, // 打开内部电荷泵
    0x20, 0x02, // 页寻址模式
    0xA1,       // 列地址翻转 TODO：显示左右颠倒时改成 0xA0
    0xC8,       // 行扫描翻转 TODO：显示上下颠倒时改成 0xC0
    0xDA, 0x12, // COM 引脚配置
    0x81, 0xCF, // 对比度
    0xD9, 0xF1, // 预充电周期
    0xDB, 0x40, // VCOMH 电平
    0xA4,       // 按显存内容显示
    0xA6,       // 正常显示，不反色
    0x2E,       // 关闭滚动
    0xAF,       // 开显示
};

bool Ssd1306_Init(Ssd1306_t *dev, const I2cBus_t *bus, uint8_t addr)
{
    if (dev == NULL)
    {
        return false;
    }

    dev->ready = false;

    if (bus == NULL || bus->write == NULL ||
        (addr != SSD1306_ADDR_LOW && addr != SSD1306_ADDR_HIGH))
    {
        return false;
    }

    if (!bus->write(bus->ctx, addr, SSD1306_CTRL_CMD,
                    INIT_CMDS, (uint16_t)sizeof(INIT_CMDS)))
    {
        return false;
    }

    dev->bus = *bus;
    dev->addr = addr;
    memset(dev->buffer, 0, sizeof(dev->buffer));

    // 屏幕上可能还留着复位前的内容，全部重新发送一遍
    dev->dirty = SSD1306_ALL_PAGES;
    dev->ready = true;
    return true;
}

bool Ssd1306_WriteLine(Ssd1306_t *dev, uint8_t page, const char *text)
{
    if (dev == NULL || text == NULL || page >= SSD1306_PAGES)
    {
        return false;
    }

    uint8_t row[SSD1306_WIDTH] = {0};
    size_t col = 0U;

    for (size_t idx = 0U; idx < SSD1306_TEXT_COLS && text[idx] != '\0'; idx++)
    {
        const uint8_t *glyph = Font5x7_Glyph(text[idx]);

        memcpy(&row[col], glyph, FONT5X7_WIDTH);
        col += FONT5X7_WIDTH + 1U; // 字符之间空 1 列
    }

    // 内容没变就不刷新，减少 I2C 传输
    if (memcmp(row, dev->buffer[page], SSD1306_WIDTH) != 0)
    {
        memcpy(dev->buffer[page], row, SSD1306_WIDTH);
        dev->dirty = (uint8_t)(dev->dirty | (1U << page));
    }

    return true;
}

/**
 * @brief 发送一页：先设页号和起始列，再分块发送 128 字节显存
 */
static bool SendPage(const Ssd1306_t *dev, uint8_t page)
{
    const uint8_t cmds[] = {
        (uint8_t)(0xB0U | page), // 页号
        0x00U,                   // 起始列低 4 位
        0x10U,                   // 起始列高 4 位
    };

    if (!dev->bus.write(dev->bus.ctx, dev->addr, SSD1306_CTRL_CMD,
                        cmds, (uint16_t)sizeof(cmds)))
    {
        return false;
    }

    for (size_t col = 0U; col < SSD1306_WIDTH; col += SSD1306_CHUNK_BYTES)
    {
        if (!dev->bus.write(dev->bus.ctx, dev->addr, SSD1306_CTRL_DATA,
                            &dev->buffer[page][col], SSD1306_CHUNK_BYTES))
        {
            return false;
        }
    }

    return true;
}

bool Ssd1306_Flush(Ssd1306_t *dev)
{
    if (dev == NULL || !dev->ready)
    {
        return false;
    }

    for (uint8_t page = 0U; page < SSD1306_PAGES; page++)
    {
        uint8_t mask = (uint8_t)(1U << page);

        if ((dev->dirty & mask) == 0U)
        {
            continue;
        }

        if (!SendPage(dev, page))
        {
            // 屏幕可能掉电或总线出错，交给调用方重新初始化
            dev->ready = false;
            return false;
        }

        dev->dirty = (uint8_t)(dev->dirty & ~mask);
    }

    return true;
}
