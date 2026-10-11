//
// Created by YZH on 2026/10/11.
//
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "handheld_packet.h"

/* 标准答案由独立的 Python 实现计算，防止编码和解码一起写错 */
static const uint8_t GOLDEN_PACKET[HH_PACKET_LEN] = {
    0x01, 0x2A, 0xE8, 0x03, 0x18, 0xFC, 0x0B, 0x02,
    0x64, 0x9C, 0x00, 0x01, 0xFF, 0x32, 0x02
};

static const uint8_t GOLDEN_STATUS[HH_STATUS_LEN] = {
    0x01, 0x03, 0x02, 0x5A
};

static HhPacket_t GoldenPacket(void)
{
    HhPacket_t packet = {
        .seq = 0x2A,
        .forward = 1000,
        .left = -1000,
        .flags = HH_FLAG_ENABLED | HH_FLAG_ESTOP | HH_FLAG_GEAR,
        .preset = HH_PRESET_STOW,
        .jog = {100, -100, 0, 1, -1, 50}
    };
    return packet;
}

static void ExpectPacket(const HhPacket_t *actual, const HhPacket_t *expected)
{
    assert(actual->seq == expected->seq);
    assert(actual->forward == expected->forward);
    assert(actual->left == expected->left);
    assert(actual->flags == expected->flags);
    assert(actual->preset == expected->preset);
    for (size_t idx = 0; idx < HH_JOINT_COUNT; ++idx)
    {
        assert(actual->jog[idx] == expected->jog[idx]);
    }
}

/* 用正确的 CRC 重新封口，用来构造“CRC 对但内容非法”的包 */
static void Reseal(uint8_t *data, size_t length)
{
    data[length - 1U] = HhPacket_Crc8(data, length - 1U);
}

static void TestCrcCheckValue(void)
{
    const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    assert(HhPacket_Crc8(check, sizeof(check)) == 0xF4U);
    assert(HhPacket_Crc8(NULL, 0U) == 0x00U);
}

static void TestEncodeMatchesGolden(void)
{
    HhPacket_t packet = GoldenPacket();
    uint8_t out[HH_PACKET_LEN + 4U];
    memset(out, 0xAA, sizeof(out));

    assert(HhPacket_Encode(&packet, out, sizeof(out)) == HH_PACKET_LEN);
    assert(memcmp(out, GOLDEN_PACKET, HH_PACKET_LEN) == 0);
    assert(out[HH_PACKET_LEN] == 0xAAU); // 不写超出 HH_PACKET_LEN 的部分
}

static void TestDecodeGolden(void)
{
    HhPacket_t expected = GoldenPacket();
    HhPacket_t actual = {0};

    assert(HhPacket_Decode(GOLDEN_PACKET, HH_PACKET_LEN, &actual));
    ExpectPacket(&actual, &expected);
}

static void TestRoundTripZeroAndNegative(void)
{
    HhPacket_t packet = {
        .seq = 255U,
        .forward = -1,
        .left = 0,
        .flags = HH_FLAG_TURN_CCW | HH_FLAG_TURN_CW | HH_FLAG_ARM_STOP,
        .preset = HH_PRESET_READY,
        .jog = {-100, 0, 0, 0, 0, 100}
    };
    uint8_t out[HH_PACKET_LEN];
    HhPacket_t decoded = {0};

    assert(HhPacket_Encode(&packet, out, sizeof(out)) == HH_PACKET_LEN);
    assert(HhPacket_Decode(out, sizeof(out), &decoded));
    ExpectPacket(&decoded, &packet);
}

static void TestEncodeRejectsInvalid(void)
{
    uint8_t out[HH_PACKET_LEN];
    HhPacket_t packet = GoldenPacket();

    assert(HhPacket_Encode(NULL, out, sizeof(out)) == 0U);
    assert(HhPacket_Encode(&packet, NULL, sizeof(out)) == 0U);
    assert(HhPacket_Encode(&packet, out, HH_PACKET_LEN - 1U) == 0U);

    packet = GoldenPacket();
    packet.forward = HH_AXIS_MAX + 1;
    assert(HhPacket_Encode(&packet, out, sizeof(out)) == 0U);

    packet = GoldenPacket();
    packet.left = -HH_AXIS_MAX - 1;
    assert(HhPacket_Encode(&packet, out, sizeof(out)) == 0U);

    packet = GoldenPacket();
    packet.jog[3] = -HH_JOG_MAX - 1;
    assert(HhPacket_Encode(&packet, out, sizeof(out)) == 0U);

    packet = GoldenPacket();
    packet.flags = 0x40U;
    assert(HhPacket_Encode(&packet, out, sizeof(out)) == 0U);

    packet = GoldenPacket();
    packet.preset = 3U;
    assert(HhPacket_Encode(&packet, out, sizeof(out)) == 0U);
}

static void TestDecodeRejectsLengthAndArgs(void)
{
    HhPacket_t packet = {0};

    assert(!HhPacket_Decode(GOLDEN_PACKET, HH_PACKET_LEN - 1U, &packet));
    assert(!HhPacket_Decode(GOLDEN_PACKET, HH_PACKET_LEN + 1U, &packet));
    assert(!HhPacket_Decode(NULL, HH_PACKET_LEN, &packet));
    assert(!HhPacket_Decode(GOLDEN_PACKET, HH_PACKET_LEN, NULL));
}

/* 任何一个字节出错都要被 CRC 拦下 */
static void TestDecodeRejectsCorruptedByte(void)
{
    for (size_t idx = 0; idx < HH_PACKET_LEN; ++idx)
    {
        uint8_t data[HH_PACKET_LEN];
        memcpy(data, GOLDEN_PACKET, sizeof(data));
        data[idx] ^= 0x01U;

        HhPacket_t packet = {0};
        assert(!HhPacket_Decode(data, sizeof(data), &packet));
    }
}

static void TestDecodeRejectsBadContentWithValidCrc(void)
{
    uint8_t data[HH_PACKET_LEN];
    HhPacket_t packet = {0};

    memcpy(data, GOLDEN_PACKET, sizeof(data));
    data[0] = HH_PACKET_VERSION + 1U;
    Reseal(data, sizeof(data));
    assert(!HhPacket_Decode(data, sizeof(data), &packet));

    memcpy(data, GOLDEN_PACKET, sizeof(data));
    data[2] = 0xE9U; // forward = 1001
    Reseal(data, sizeof(data));
    assert(!HhPacket_Decode(data, sizeof(data), &packet));

    memcpy(data, GOLDEN_PACKET, sizeof(data));
    data[6] = 0x80U; // 未定义的标志位
    Reseal(data, sizeof(data));
    assert(!HhPacket_Decode(data, sizeof(data), &packet));

    memcpy(data, GOLDEN_PACKET, sizeof(data));
    data[7] = 3U; // 未定义的预设
    Reseal(data, sizeof(data));
    assert(!HhPacket_Decode(data, sizeof(data), &packet));

    memcpy(data, GOLDEN_PACKET, sizeof(data));
    data[13] = 0x9BU; // jog = -101
    Reseal(data, sizeof(data));
    assert(!HhPacket_Decode(data, sizeof(data), &packet));
}

/* 解码失败时不能改动输出，调用方可以继续使用上一帧 */
static void TestDecodeFailureKeepsOutput(void)
{
    HhPacket_t packet = GoldenPacket();
    HhPacket_t before = packet;
    uint8_t data[HH_PACKET_LEN];

    memcpy(data, GOLDEN_PACKET, sizeof(data));
    data[HH_PACKET_LEN - 1U] ^= 0xFFU;
    assert(!HhPacket_Decode(data, sizeof(data), &packet));
    ExpectPacket(&packet, &before);
}

static void TestStatusGoldenAndRoundTrip(void)
{
    HhStatus_t status = {
        .flags = HH_STATUS_ENABLED | HH_STATUS_SAFETY,
        .gear = 2U
    };
    uint8_t out[HH_STATUS_LEN];
    HhStatus_t decoded = {0};

    assert(HhStatus_Encode(&status, out, sizeof(out)) == HH_STATUS_LEN);
    assert(memcmp(out, GOLDEN_STATUS, HH_STATUS_LEN) == 0);
    assert(HhStatus_Decode(out, sizeof(out), &decoded));
    assert(decoded.flags == status.flags);
    assert(decoded.gear == status.gear);
}

static void TestStatusRejectsInvalid(void)
{
    uint8_t out[HH_STATUS_LEN];
    HhStatus_t status = {.flags = 0x04U, .gear = 1U};
    HhStatus_t decoded = {.gear = 9U};

    // 未定义的状态位
    assert(HhStatus_Encode(&status, out, sizeof(out)) == 0U);

    status.flags = 0U;
    assert(HhStatus_Encode(&status, out, HH_STATUS_LEN - 1U) == 0U);
    assert(HhStatus_Encode(NULL, out, sizeof(out)) == 0U);

    assert(HhStatus_Encode(&status, out, sizeof(out)) == HH_STATUS_LEN);
    out[2] ^= 0x01U;
    assert(!HhStatus_Decode(out, sizeof(out), &decoded));
    assert(decoded.gear == 9U);
    assert(!HhStatus_Decode(GOLDEN_STATUS, HH_STATUS_LEN - 1U, &decoded));
    assert(!HhStatus_Decode(GOLDEN_STATUS, HH_STATUS_LEN, NULL));
}

int main(void)
{
    TestCrcCheckValue();
    TestEncodeMatchesGolden();
    TestDecodeGolden();
    TestRoundTripZeroAndNegative();
    TestEncodeRejectsInvalid();
    TestDecodeRejectsLengthAndArgs();
    TestDecodeRejectsCorruptedByte();
    TestDecodeRejectsBadContentWithValidCrc();
    TestDecodeFailureKeepsOutput();
    TestStatusGoldenAndRoundTrip();
    TestStatusRejectsInvalid();

    puts("Handheld packet tests passed");
    return 0;
}
