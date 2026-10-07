//
// Created by YZH on 2026/10/6.
//

#ifndef XIAOSAI_OLED_SCREEN_H
#define XIAOSAI_OLED_SCREEN_H

#include <stdbool.h>
#include <stdint.h>

#include "ssd1306.h"

// 字库收录的字符范围，必须和 font5x7.c 保持一致
#define OLED_SCREEN_FIRST_CHAR ' '
#define OLED_SCREEN_LAST_CHAR  'Z'
#define OLED_SCREEN_GLYPH_COUNT \
    ((unsigned)(OLED_SCREEN_LAST_CHAR - OLED_SCREEN_FIRST_CHAR + 1))

// 第 1～3 页依次显示字库里的全部字符
#define OLED_SCREEN_CHARSET_FIRST_PAGE 1U
#define OLED_SCREEN_CHARSET_LAST_PAGE  3U

// 一行文字的缓冲区大小，含结尾的 '\0'
#define OLED_SCREEN_LINE_SIZE (SSD1306_TEXT_COLS + 1U)

/**
 * @brief 生成测试画面第 page 行的文字
 * @param page 0～7：标题、3 行全部字符、列标尺、字符范围、I2C 地址、运行时间
 * @param addr 屏幕的 7 位 I2C 地址，显示在第 6 页
 * @param uptime_s 上电后的秒数，显示在第 7 页
 * @param out 至少 OLED_SCREEN_LINE_SIZE 字节，输出以 '\0' 结尾，不超过 21 个字符
 * @return page 有效且 out 非空返回 true
 * @note 只生成文字，不访问屏幕；画到屏幕上用 Ssd1306_WriteLine
 */
bool OledScreen_BuildLine(uint8_t page, uint8_t addr, uint32_t uptime_s,
                          char out[OLED_SCREEN_LINE_SIZE]);

#endif //XIAOSAI_OLED_SCREEN_H
