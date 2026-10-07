//
// Created by YZH on 2026/10/6.
//

#include "oled_screen.h"

#include <stddef.h>

#define PAGE_TITLE  0U
#define PAGE_RULER  4U
#define PAGE_RANGE  5U
#define PAGE_ADDR   6U
#define PAGE_UPTIME 7U

#define SECONDS_PER_MINUTE 60UL
#define SECONDS_PER_HOUR   3600UL

// uint32_t 最多 10 位十进制
#define UINT32_MAX_DIGITS 10U

_Static_assert(OLED_SCREEN_GLYPH_COUNT <=
                   (OLED_SCREEN_CHARSET_LAST_PAGE - OLED_SCREEN_CHARSET_FIRST_PAGE + 1U) *
                       SSD1306_TEXT_COLS,
               "charset pages cannot hold every glyph");

static const char HEX_DIGITS[] = "0123456789ABCDEF";

/**
 * @brief 往一行里追加文字，写满 SSD1306_TEXT_COLS 个字符后多出来的丢掉
 */
typedef struct
{
    char *buf;
    size_t len;
} Line_t;

static void AppendChar(Line_t *line, char c)  // 后加字符
{
    if (line->len < SSD1306_TEXT_COLS)
    {
        line->buf[line->len++] = c;
        line->buf[line->len] = '\0';
    }
}

static void AppendStr(Line_t *line, const char *text)  // 后加字符串
{
    for (size_t idx = 0U; text[idx] != '\0'; idx++)
    {
        AppendChar(line, text[idx]);
    }
}

/**
 * @brief 追加十进制数，不足 min_digits 位时前面补 0
 */
static void AppendUint(Line_t *line, uint32_t value, size_t min_digits)
{
    char reversed[UINT32_MAX_DIGITS];
    size_t count = 0U;

    do
    {
        reversed[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value > 0U);

    while (count < min_digits && count < UINT32_MAX_DIGITS)
    {
        reversed[count++] = '0';
    }

    while (count > 0U)
    {
        AppendChar(line, reversed[--count]);
    }
}

/**
 * @brief 追加两位十六进制数，例如 0x3C → "0X3C"
 * @note 字库没有小写字母，前缀直接写成大写，和屏幕上看到的一致
 */
static void AppendHex8(Line_t *line, uint8_t value)
{
    AppendStr(line, "0X");
    AppendChar(line, HEX_DIGITS[value >> 4]);
    AppendChar(line, HEX_DIGITS[value & 0x0FU]);
}

/**
 * @brief 第 1～3 页：从空格开始按顺序排字库里的字符，每页 21 个
 */
static void BuildCharset(Line_t *line, uint8_t page)
{
    unsigned first = (unsigned)(page - OLED_SCREEN_CHARSET_FIRST_PAGE) * SSD1306_TEXT_COLS;

    for (unsigned idx = first;
         idx < OLED_SCREEN_GLYPH_COUNT && idx < first + SSD1306_TEXT_COLS; idx++)
    {
        AppendChar(line, (char)(OLED_SCREEN_FIRST_CHAR + (int)idx));
    }
}

/**
 * @brief 列标尺 "123456789012345678901"，最右一列缺了说明屏幕列偏移不对
 */
static void BuildRuler(Line_t *line)
{
    for (unsigned col = 1U; col <= SSD1306_TEXT_COLS; col++)
    {
        AppendChar(line, (char)('0' + col % 10U));
    }
}

static void BuildRange(Line_t *line)
{
    AppendUint(line, OLED_SCREEN_GLYPH_COUNT, 1U);
    AppendStr(line, " GLYPHS ");
    AppendHex8(line, (uint8_t)OLED_SCREEN_FIRST_CHAR);
    AppendChar(line, '-');
    AppendHex8(line, (uint8_t)OLED_SCREEN_LAST_CHAR);
}

/**
 * @brief 运行时间，格式为 时:分:秒，小时至少两位
 */
static void BuildUptime(Line_t *line, uint32_t uptime_s)
{
    AppendStr(line, "UPTIME ");
    AppendUint(line, (uint32_t)(uptime_s / SECONDS_PER_HOUR), 2U);
    AppendChar(line, ':');
    AppendUint(line, (uint32_t)(uptime_s / SECONDS_PER_MINUTE % 60UL), 2U);
    AppendChar(line, ':');
    AppendUint(line, (uint32_t)(uptime_s % SECONDS_PER_MINUTE), 2U);
}

bool OledScreen_BuildLine(uint8_t page, uint8_t addr, uint32_t uptime_s,
                          char out[OLED_SCREEN_LINE_SIZE])
{
    if (out == NULL || page >= SSD1306_PAGES)
    {
        return false;
    }

    Line_t line = {.buf = out, .len = 0U};
    out[0] = '\0';

    if (page >= OLED_SCREEN_CHARSET_FIRST_PAGE && page <= OLED_SCREEN_CHARSET_LAST_PAGE)
    {
        BuildCharset(&line, page);
        return true;
    }

    switch (page)
    {
    case PAGE_TITLE:
        AppendStr(&line, "OLED FONT5X7 TEST");
        break;
    case PAGE_RULER:
        BuildRuler(&line);
        break;
    case PAGE_RANGE:
        BuildRange(&line);
        break;
    case PAGE_ADDR:
        AppendStr(&line, "I2C2 ADDR ");
        AppendHex8(&line, addr);
        break;
    default: // PAGE_UPTIME
        BuildUptime(&line, uptime_s);
        break;
    }

    return true;
}
