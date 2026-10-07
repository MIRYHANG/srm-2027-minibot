//
// Created by YZH on 2026/10/6.
//
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "font5x7.h"
#include "oled_screen.h"

#define PAGE_TITLE   0U
#define PAGE_RULER   4U
#define PAGE_RANGE   5U
#define PAGE_ADDR    6U
#define PAGE_UPTIME  7U

static void BuildLine(uint8_t page, uint8_t addr, uint32_t uptime_s,
                      char out[OLED_SCREEN_LINE_SIZE])
{
    assert(OledScreen_BuildLine(page, addr, uptime_s, out));
    assert(strlen(out) <= SSD1306_TEXT_COLS);
}

static void AssertLine(uint8_t page, uint8_t addr, uint32_t uptime_s,
                       const char *expected)
{
    char line[OLED_SCREEN_LINE_SIZE];

    BuildLine(page, addr, uptime_s, line);
    if (strcmp(line, expected) != 0)
    {
        printf("page %u: got \"%s\", expected \"%s\"\n",
               (unsigned)page, line, expected);
        assert(false);
    }
}

/* 字符范围必须和 font5x7.c 收录的范围一致：范围外显示为 '?'，范围内都有自己的字形 */
static void TestRangeMatchesFont(void)
{
    const uint8_t *fallback = Font5x7_Glyph('?');

    assert(Font5x7_Glyph((char)(OLED_SCREEN_FIRST_CHAR - 1)) == fallback);
    assert(Font5x7_Glyph((char)(OLED_SCREEN_LAST_CHAR + 1)) == fallback);

    for (int c = OLED_SCREEN_FIRST_CHAR; c <= OLED_SCREEN_LAST_CHAR; c++)
    {
        assert(c == '?' || Font5x7_Glyph((char)c) != fallback);
    }
}

/* 字符页按顺序拼起来正好是全部字符，每个只出现一次 */
static void TestCharsetPages(void)
{
    char all[OLED_SCREEN_GLYPH_COUNT + 1U] = {0};
    size_t len = 0U;

    for (uint8_t page = 0U; page < SSD1306_PAGES; page++)
    {
        if (page < OLED_SCREEN_CHARSET_FIRST_PAGE ||
            page > OLED_SCREEN_CHARSET_LAST_PAGE)
        {
            continue;
        }

        char line[OLED_SCREEN_LINE_SIZE];
        BuildLine(page, SSD1306_ADDR_LOW, 0U, line);

        size_t line_len = strlen(line);
        assert(len + line_len <= OLED_SCREEN_GLYPH_COUNT);
        memcpy(&all[len], line, line_len);
        len += line_len;
    }

    assert(len == OLED_SCREEN_GLYPH_COUNT);
    for (size_t idx = 0U; idx < len; idx++)
    {
        assert(all[idx] == (char)(OLED_SCREEN_FIRST_CHAR + (int)idx));
    }

    // 前面的字符页排满一整行
    char first[OLED_SCREEN_LINE_SIZE];
    BuildLine(OLED_SCREEN_CHARSET_FIRST_PAGE, SSD1306_ADDR_LOW, 0U, first);
    assert(strlen(first) == SSD1306_TEXT_COLS);
}

/* 其他页只用字库里有的字符，不会意外显示成 '?' */
static void TestOtherPagesUseFontChars(void)
{
    for (uint8_t page = 0U; page < SSD1306_PAGES; page++)
    {
        if (page >= OLED_SCREEN_CHARSET_FIRST_PAGE &&
            page <= OLED_SCREEN_CHARSET_LAST_PAGE)
        {
            continue;
        }

        char line[OLED_SCREEN_LINE_SIZE];
        BuildLine(page, SSD1306_ADDR_HIGH, 123456U, line);
        assert(strlen(line) > 0U);

        for (size_t idx = 0U; line[idx] != '\0'; idx++)
        {
            assert(line[idx] >= OLED_SCREEN_FIRST_CHAR &&
                   line[idx] <= OLED_SCREEN_LAST_CHAR &&
                   line[idx] != '?');
        }
    }
}

static void TestFixedPages(void)
{
    char title[OLED_SCREEN_LINE_SIZE];
    BuildLine(PAGE_TITLE, SSD1306_ADDR_LOW, 0U, title);
    assert(strlen(title) > 0U);

    // 标尺占满 21 列，用来检查最右边一列有没有被截掉
    AssertLine(PAGE_RULER, SSD1306_ADDR_LOW, 0U, "123456789012345678901");
    AssertLine(PAGE_RANGE, SSD1306_ADDR_LOW, 0U, "59 GLYPHS 0X20-0X5A");
}

static void TestAddrPage(void)
{
    AssertLine(PAGE_ADDR, SSD1306_ADDR_LOW, 0U, "I2C2 ADDR 0X3C");
    AssertLine(PAGE_ADDR, SSD1306_ADDR_HIGH, 0U, "I2C2 ADDR 0X3D");
}

static void TestUptimePage(void)
{
    AssertLine(PAGE_UPTIME, SSD1306_ADDR_LOW, 0U, "UPTIME 00:00:00");
    AssertLine(PAGE_UPTIME, SSD1306_ADDR_LOW, 59U, "UPTIME 00:00:59");
    AssertLine(PAGE_UPTIME, SSD1306_ADDR_LOW, 3661U, "UPTIME 01:01:01");
    AssertLine(PAGE_UPTIME, SSD1306_ADDR_LOW, 359999U, "UPTIME 99:59:59");
    AssertLine(PAGE_UPTIME, SSD1306_ADDR_LOW, 360000U, "UPTIME 100:00:00");
    AssertLine(PAGE_UPTIME, SSD1306_ADDR_LOW, UINT32_MAX, "UPTIME 1193046:28:15");
}

static void TestInvalidArgs(void)
{
    char line[OLED_SCREEN_LINE_SIZE];

    assert(!OledScreen_BuildLine(SSD1306_PAGES, SSD1306_ADDR_LOW, 0U, line));
    assert(!OledScreen_BuildLine(PAGE_TITLE, SSD1306_ADDR_LOW, 0U, NULL));
}

int main(void)
{
    TestRangeMatchesFont();
    TestCharsetPages();
    TestOtherPagesUseFontChars();
    TestFixedPages();
    TestAddrPage();
    TestUptimePage();
    TestInvalidArgs();

    puts("OLED screen tests passed");
    return 0;
}
