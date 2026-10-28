//
// Created by YZH on 2026/10/5.
//

#include "power_display.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

// 数值部分统一 7 个字符宽，单位对齐在同一列
#define VALUE_WIDTH 7U

// 超过这个数（按最小单位计）就认为数据异常，显示横线
#define VALUE_MAX_SCALED 999999L

/**
 * @brief 生成和数值同样形状的横线，例如 2 位小数为 "--.--"，右对齐
 */
static void FormatDashes(char out[VALUE_WIDTH + 1U], unsigned decimals)
{
    memset(out, ' ', VALUE_WIDTH);
    out[VALUE_WIDTH] = '\0';

    size_t pos = VALUE_WIDTH;
    for (unsigned d = 0U; d < decimals; d++)
    {
        out[--pos] = '-';
    }
    if (decimals > 0U)
    {
        out[--pos] = '.';
    }
    out[--pos] = '-';
    out[--pos] = '-';
}

/**
 * @brief 把数值格式化成右对齐的定点小数，例如 12.34 → "  12.34"
 * @note 非有限值或放不下时显示横线；四舍五入后为 0 的负数不显示负号
 */
static void FormatFixed(char out[VALUE_WIDTH + 1U], float value, unsigned decimals)
{
    float scale = 1.0f;
    for (unsigned d = 0U; d < decimals; d++)
    {
        scale *= 10.0f;
    }

    float magnitude = fabsf(value) * scale;
    if (!isfinite(value) || magnitude > (float)VALUE_MAX_SCALED)
    {
        FormatDashes(out, decimals);
        return;
    }

    long scaled = lroundf(magnitude);
    char reversed[VALUE_WIDTH + 2U];
    size_t len = 0U;

    // 从个位开始倒着生成：先小数位，再小数点，再整数位
    for (unsigned d = 0U; d < decimals; d++)
    {
        reversed[len++] = (char)('0' + scaled % 10L);
        scaled /= 10L;
    }
    if (decimals > 0U)
    {
        reversed[len++] = '.';
    }
    do
    {
        reversed[len++] = (char)('0' + scaled % 10L);
        scaled /= 10L;
    } while (scaled > 0L && len < sizeof(reversed));

    if (value < 0.0f && lroundf(magnitude) != 0L && len < sizeof(reversed))
    {
        reversed[len++] = '-';
    }

    if (len > VALUE_WIDTH)
    {
        FormatDashes(out, decimals);
        return;
    }

    memset(out, ' ', VALUE_WIDTH);
    out[VALUE_WIDTH] = '\0';
    for (size_t idx = 0U; idx < len; idx++)
    {
        out[VALUE_WIDTH - 1U - idx] = reversed[idx];
    }
}

/**
 * @brief 依次拼接字符串，最多 POWER_DISPLAY_COLS 个字符
 */
static void Compose(PowerDisplayLine_t line, const char *parts[], size_t count)
{
    size_t len = 0U;

    for (size_t idx = 0U; idx < count; idx++)
    {
        for (const char *p = parts[idx]; *p != '\0' && len < POWER_DISPLAY_COLS; p++)
        {
            line[len++] = *p;
        }
    }

    line[len] = '\0';
}

static void ComposeValue(PowerDisplayLine_t line, const char *label, float value,
                         unsigned decimals, bool valid, const char *unit)
{
    char number[VALUE_WIDTH + 1U];

    if (valid)
    {
        FormatFixed(number, value, decimals);
    }
    else
    {
        FormatDashes(number, decimals);
    }

    const char *parts[] = {label, " ", number, " ", unit};
    Compose(line, parts, sizeof(parts) / sizeof(parts[0]));
}

static const char *ResetCauseText(ResetCause_t cause)
{
    switch (cause)
    {
    case RESET_CAUSE_POWER:     return "RST POWER";
    case RESET_CAUSE_PIN:       return "RST PIN";
    case RESET_CAUSE_SOFTWARE:  return "RST SOFT";
    case RESET_CAUSE_IWDG:      return "RST IWDG";
    case RESET_CAUSE_WWDG:      return "RST WWDG";
    case RESET_CAUSE_LOW_POWER: return "RST LOWPWR";
    default:                    return "RST ?";
    }
}

void PowerDisplay_Format(const Ina226Reading_t *reading, bool valid,
                         ResetCause_t cause,
                         PowerDisplayLine_t lines[POWER_DISPLAY_LINES])
{
    if (lines == NULL)
    {
        return;
    }

    bool has_data = valid && reading != NULL;
    Ina226Reading_t data = has_data ? *reading : (Ina226Reading_t){0};

    for (size_t idx = 0U; idx < POWER_DISPLAY_LINES; idx++)
    {
        lines[idx][0] = '\0';
    }

    const char *title[] = {"XIAOSAI POWER"};
    Compose(lines[0], title, 1U);

    ComposeValue(lines[2], "U", data.bus_v, 2U, has_data, "V");
    ComposeValue(lines[3], "I", data.current_a, 2U, has_data, "A");
    ComposeValue(lines[4], "P", data.power_w, 1U, has_data, "W");

    const char *status[] = {has_data ? "INA226 OK" : "INA226 ERROR"};
    Compose(lines[6], status, 1U);

    const char *reset[] = {ResetCauseText(cause)};
    Compose(lines[7], reset, 1U);
}

static const char *BatteryLevelText(BatteryLevel_t level)
{
    switch (level)
    {
    case BATTERY_OK:       return "OK";
    case BATTERY_LOW:      return "LOW";
    case BATTERY_CRITICAL: return "STOP";
    default:               return "?";
    }
}

void PowerDisplay_FormatBattery(BatteryLevel_t level, uint8_t percent,
                                PowerDisplayLine_t line)
{
    if (line == NULL)
    {
        return;
    }

    // 电量固定 3 位右对齐，和状态之间留一个空格
    char number[4] = {' ', ' ', '-', '\0'};
    if (level != BATTERY_UNKNOWN)
    {
        uint8_t value = percent > 100U ? 100U : percent;
        number[0] = value >= 100U ? '1' : ' ';
        number[1] = value >= 10U ? (char)('0' + (value / 10U) % 10U) : ' ';
        number[2] = (char)('0' + value % 10U);
    }

    const char *parts[] = {"BAT ", number, "% ", BatteryLevelText(level)};
    Compose(line, parts, sizeof(parts) / sizeof(parts[0]));
}

/**
 * @brief 取后三位，补零到 3 位，例如 1045 → "045"
 */
static void FormatLast3(char out[4], uint32_t value)
{
    value %= 1000U;
    out[0] = (char)('0' + value / 100U);
    out[1] = (char)('0' + (value / 10U) % 10U);
    out[2] = (char)('0' + value % 10U);
    out[3] = '\0';
}

void PowerDisplay_FormatRemote(const PowerDisplayRemote_t *remote,
                               PowerDisplayLine_t line)
{
    if (line == NULL)
    {
        return;
    }

    if (remote == NULL)
    {
        line[0] = '\0';
        return;
    }

    static const char HEX[] = "0123456789ABCDEF";

    char bytes[4];
    char frames[4];
    FormatLast3(bytes, remote->rx_bytes);
    FormatLast3(frames, remote->rx_frames);

    char switches[3] = {'-', '-', '\0'};
    if (remote->online)
    {
        switches[0] = remote->s1 ? '1' : '0';
        switches[1] = remote->s2 ? '1' : '0';
    }

    char safety[3] = {HEX[(remote->safety >> 4) & 0x0FU], HEX[remote->safety & 0x0FU], '\0'};

    const char *parts[] = {"R", bytes, " F", frames, " S", switches,
                           " X", safety, " E", remote->enabled ? "1" : "0"};
    Compose(line, parts, sizeof(parts) / sizeof(parts[0]));
}

void PowerDisplay_FormatArm(const PowerDisplayRemote_t *remote, uint16_t pulse_us,
                            PowerDisplayLine_t line)
{
    if (line == NULL)
    {
        return;
    }

    if (remote == NULL)
    {
        line[0] = '\0';
        return;
    }

    char joint[2] = {'?', '\0'};
    if (remote->arm_joint >= 1U && remote->arm_joint <= 9U)
    {
        joint[0] = (char)('0' + remote->arm_joint);
    }

    // jog 显示为带符号的 3 位百分比，非法值显示横线
    char jog[5] = {' ', '-', '-', '-', '\0'};
    if (isfinite(remote->arm_jog) && fabsf(remote->arm_jog) <= 1.0f)
    {
        long percent = lroundf(remote->arm_jog * 100.0f);
        jog[0] = percent < 0L ? '-' : '+';
        FormatLast3(&jog[1], (uint32_t)labs(percent));
    }

    char pulse[5];
    FormatLast3(&pulse[1], pulse_us);
    pulse[0] = (char)('0' + (pulse_us / 1000U) % 10U);

    const char *parts[] = {"J", joint, " G", jog, " P", pulse,
                           " A", remote->arm_enabled ? "1" : "0"};
    Compose(line, parts, sizeof(parts) / sizeof(parts[0]));
}
