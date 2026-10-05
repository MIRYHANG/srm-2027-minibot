//
// Created by YZH on 2026/10/5.
//
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "font5x7.h"
#include "ssd1306.h"

#define MAX_WRITES 128

/* 假总线：记录每一次写入的控制字节、长度和前 32 个字节 */
typedef struct
{
    uint8_t ctrl;
    uint16_t len;
    uint8_t data[32];
} WriteRecord_t;

typedef struct
{
    uint8_t addr;
    WriteRecord_t writes[MAX_WRITES];
    int count;
    int fail_at; // 第几次写入失败，-1 表示不失败
} FakeOled_t;

static bool FakeWrite(void *ctx, uint8_t addr7, uint8_t reg,
                      const uint8_t *data, uint16_t len)
{
    FakeOled_t *fake = ctx;

    if (addr7 != fake->addr || fake->count == fake->fail_at)
    {
        fake->count++;
        return false;
    }

    assert(fake->count < MAX_WRITES);
    WriteRecord_t *rec = &fake->writes[fake->count++];
    rec->ctrl = reg;
    rec->len = len;
    memcpy(rec->data, data, len < sizeof(rec->data) ? len : sizeof(rec->data));
    return true;
}

static I2cBus_t BusOf(FakeOled_t *fake)
{
    return (I2cBus_t){.read = NULL, .write = FakeWrite, .ctx = fake};
}

static void ResetFake(FakeOled_t *fake)
{
    memset(fake, 0, sizeof(*fake));
    fake->addr = SSD1306_ADDR_LOW;
    fake->fail_at = -1;
}

/* 显存有 1 KB，和固件一样用静态变量 */
static Ssd1306_t oled;
static FakeOled_t fake;

static void InitOled(void)
{
    ResetFake(&fake);
    I2cBus_t bus = BusOf(&fake);
    assert(Ssd1306_Init(&oled, &bus, SSD1306_ADDR_LOW));
}

/* 检查一次完整的页发送：1 次命令 + 8 次 16 字节数据 */
static void AssertPageSent(int first, uint8_t page)
{
    const WriteRecord_t *cmd = &fake.writes[first];
    assert(cmd->ctrl == 0x00U);
    assert(cmd->len == 3U);
    assert(cmd->data[0] == (uint8_t)(0xB0U | page));
    assert(cmd->data[1] == 0x00U);
    assert(cmd->data[2] == 0x10U);

    for (int chunk = 0; chunk < 8; chunk++)
    {
        const WriteRecord_t *rec = &fake.writes[first + 1 + chunk];
        assert(rec->ctrl == 0x40U);
        assert(rec->len == 16U);
        assert(memcmp(rec->data, &oled.buffer[page][chunk * 16], 16U) == 0);
    }
}

static void TestFont(void)
{
    static const uint8_t zero[] = {0x3E, 0x51, 0x49, 0x45, 0x3E};
    assert(memcmp(Font5x7_Glyph('0'), zero, sizeof(zero)) == 0);

    // 小写按大写显示，未收录的字符显示为 '?'
    assert(Font5x7_Glyph('a') == Font5x7_Glyph('A'));
    assert(Font5x7_Glyph('~') == Font5x7_Glyph('?'));
    assert(Font5x7_Glyph('\x01') == Font5x7_Glyph('?'));
    assert(Font5x7_Glyph(' ')[0] == 0x00U);
}

static void TestInit(void)
{
    InitOled();

    // 初始化命令一次发完：以关显示开头、开显示结尾，并打开电荷泵
    assert(fake.count == 1);
    const WriteRecord_t *rec = &fake.writes[0];
    assert(rec->ctrl == 0x00U);
    assert(rec->data[0] == 0xAEU);
    assert(rec->data[rec->len - 1U] == 0xAFU);

    bool charge_pump = false;
    for (uint16_t idx = 0U; idx + 1U < rec->len; idx++)
    {
        charge_pump = charge_pump || (rec->data[idx] == 0x8DU && rec->data[idx + 1U] == 0x14U);
    }
    assert(charge_pump);

    assert(oled.ready);
    assert(oled.dirty == 0xFFU);
    assert(oled.addr == SSD1306_ADDR_LOW);

    // 非法参数和写入失败：ready 为 false
    I2cBus_t bus = BusOf(&fake);
    assert(!Ssd1306_Init(NULL, &bus, SSD1306_ADDR_LOW));
    assert(!Ssd1306_Init(&oled, &bus, 0x40U));
    assert(!oled.ready);
    assert(!Ssd1306_Init(&oled, NULL, SSD1306_ADDR_LOW));

    I2cBus_t broken = bus;
    broken.write = NULL;
    assert(!Ssd1306_Init(&oled, &broken, SSD1306_ADDR_LOW));

    ResetFake(&fake);
    fake.fail_at = 0;
    assert(!Ssd1306_Init(&oled, &bus, SSD1306_ADDR_LOW));
    assert(!oled.ready);

    // 0x3D 也是合法地址
    ResetFake(&fake);
    fake.addr = SSD1306_ADDR_HIGH;
    assert(Ssd1306_Init(&oled, &bus, SSD1306_ADDR_HIGH));
}

static void TestWriteLine(void)
{
    InitOled();
    assert(Ssd1306_Flush(&oled));
    assert(oled.dirty == 0U);

    // 第 1 个字符在第 0～4 列，第 5 列空白，第 2 个字符从第 6 列开始
    assert(Ssd1306_WriteLine(&oled, 2U, "0A"));
    assert(memcmp(&oled.buffer[2][0], Font5x7_Glyph('0'), 5U) == 0);
    assert(oled.buffer[2][5] == 0x00U);
    assert(memcmp(&oled.buffer[2][6], Font5x7_Glyph('A'), 5U) == 0);
    assert(oled.dirty == (1U << 2));

    // 内容相同不标记刷新
    assert(Ssd1306_Flush(&oled));
    assert(Ssd1306_WriteLine(&oled, 2U, "0A"));
    assert(oled.dirty == 0U);

    // 整行重画：短文字会清掉旧内容
    assert(Ssd1306_WriteLine(&oled, 2U, "0"));
    assert(oled.buffer[2][6] == 0x00U);
    assert(oled.dirty == (1U << 2));

    // 超过 21 个字符时截掉，最后两列保持空白
    assert(Ssd1306_WriteLine(&oled, 7U, "8888888888888888888888888"));
    assert(memcmp(&oled.buffer[7][120], Font5x7_Glyph('8'), 5U) == 0);
    assert(oled.buffer[7][126] == 0x00U);
    assert(oled.buffer[7][127] == 0x00U);

    assert(!Ssd1306_WriteLine(&oled, 8U, "X"));
    assert(!Ssd1306_WriteLine(&oled, 0U, NULL));
    assert(!Ssd1306_WriteLine(NULL, 0U, "X"));
}

static void TestFlush(void)
{
    // 初始化后第一次刷新：8 页全部发送，共 72 次写入
    InitOled();
    assert(Ssd1306_WriteLine(&oled, 0U, "XIAOSAI"));
    fake.count = 0;
    assert(Ssd1306_Flush(&oled));
    assert(fake.count == 72);
    for (uint8_t page = 0U; page < SSD1306_PAGES; page++)
    {
        AssertPageSent(page * 9, page);
    }
    assert(oled.dirty == 0U);

    // 只改一行：只发送这一页
    assert(Ssd1306_WriteLine(&oled, 3U, "12.34"));
    fake.count = 0;
    assert(Ssd1306_Flush(&oled));
    assert(fake.count == 9);
    AssertPageSent(0, 3U);

    // 没有变化：不发送
    fake.count = 0;
    assert(Ssd1306_Flush(&oled));
    assert(fake.count == 0);

    // 发送中途失败：该页保持待刷新，ready 清零，之后不再发送
    assert(Ssd1306_WriteLine(&oled, 5U, "ERR"));
    fake.count = 0;
    fake.fail_at = 4;
    assert(!Ssd1306_Flush(&oled));
    assert(!oled.ready);
    assert(oled.dirty == (1U << 5));

    fake.fail_at = -1;
    fake.count = 0;
    assert(!Ssd1306_Flush(&oled));
    assert(fake.count == 0);
    assert(!Ssd1306_Flush(NULL));

    // 发页命令时就失败：同样保持待刷新
    InitOled();
    fake.count = 0;
    fake.fail_at = 0;
    assert(!Ssd1306_Flush(&oled));
    assert(!oled.ready);
    assert(oled.dirty == 0xFFU);
}

int main(void)
{
    TestFont();
    TestInit();
    TestWriteLine();
    TestFlush();

    puts("SSD1306 tests passed");
    return 0;
}
