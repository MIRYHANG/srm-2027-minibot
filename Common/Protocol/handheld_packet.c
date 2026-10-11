//
// Created by YZH on 2026/10/11.
//

#include "handheld_packet.h"
#define CRC8_POLY           0x07
#define PKT_VERSION         0U
#define PKT_SEQ             1U
#define PKT_FORWARD         2U   // 占 2、3 两个字节
#define PKT_LEFT            4U   // 占 4、5 两个字节
#define PKT_FLAGS           6U
#define PKT_PRESET          7U
#define PKT_JOG             8U   // 占 8～13，共 6 个字节
#define PKT_CRC             (HH_PACKET_LEN - 1U)

static void PutInt16(uint8_t *out, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    out[0] = (uint8_t)(raw & 0xFF);
    out[1] = (uint8_t)(raw >> 8);
}

static int16_t GetInt16(const uint8_t *in)
{
    uint16_t raw = (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
    return raw >= 0x8000U ? (int16_t)((int32_t)raw - 0x10000) : (int16_t)raw;
}

/**
 * @brief 检查控制包各字段是否在协议允许的范围内
 * @return 全部有效返回 true，任一字段越界或含未定义的标志位返回 false
 * @note 编码和解码共用，保证发出去的包和收下来的包遵守同一套规则
 */
static bool PacketValid(const HhPacket_t *packet)
{
    if (packet->forward < -HH_AXIS_MAX || packet->forward > HH_AXIS_MAX ||
        packet->left < -HH_AXIS_MAX || packet->left > HH_AXIS_MAX)
    {
        return false;
    }

    if ((packet->flags & (uint8_t)~HH_FLAG_VALID_MASK) != 0U)
    {
        return false;
    }

    if (packet->preset > HH_PRESET_STOW)
    {
        return false;
    }

    for (size_t idx = 0;idx < HH_JOINT_COUNT;++idx)
    {
        if (packet->jog[idx] < -HH_JOG_MAX || packet->jog[idx] > HH_JOG_MAX)
        {
            return false;
        }
    }

    return true;
}

uint8_t HhPacket_Crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;
    if (data == NULL)
    {
        return 0;
    }

    for (size_t idx = 0;idx < length;++idx)
    {
        crc ^= data[idx];
        for (int bit = 0;bit < 8;++bit)
        {
            if ((crc & 0x80) != 0x00) // 最高位是1
            {
                crc = (uint8_t)((crc << 1) ^ CRC8_POLY);
            }
            else
            {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

size_t HhPacket_Encode(const HhPacket_t *packet, uint8_t *out, size_t capacity)
{
    if (packet == NULL || out == NULL || capacity < HH_PACKET_LEN || !PacketValid(packet))
    {
        return 0U;
    }

    out[PKT_VERSION] = HH_PACKET_VERSION;
    out[PKT_SEQ] = packet->seq;
    PutInt16(&out[PKT_FORWARD], packet->forward);
    PutInt16(&out[PKT_LEFT], packet->left);
    out[PKT_FLAGS] = packet->flags;
    out[PKT_PRESET] = packet->preset;

    for (size_t idx = 0;idx < HH_JOINT_COUNT;++idx)
    {
        out[PKT_JOG + idx] = (uint8_t)packet->jog[idx];
    }

    out[PKT_CRC] = HhPacket_Crc8(out, PKT_CRC);

    return HH_PACKET_LEN;
}

bool HhPacket_Decode(const uint8_t *data, size_t length, HhPacket_t *packet)
{
    if (data == NULL || length != HH_PACKET_LEN || packet == NULL)
    {
        return false;
    }

    if (data[PKT_VERSION] != HH_PACKET_VERSION)
    {
        return false;
    }

    if (HhPacket_Crc8(data,PKT_CRC) != data[PKT_CRC])
    {
        return false;
    }
    HhPacket_t next = {0};
    next.seq = data[PKT_SEQ];
    next.forward = GetInt16(&data[PKT_FORWARD]);
    next.left = GetInt16(&data[PKT_LEFT]);
    next.flags = data[PKT_FLAGS];
    next.preset = data[PKT_PRESET];

    for (size_t idx = 0;idx < HH_JOINT_COUNT;++idx) {
        uint8_t raw = data[PKT_JOG + idx];
        next.jog[idx] = raw > 0x80 ? (int8_t)((int16_t)raw - 0x100) : (int8_t)raw;
    }

    if (!PacketValid(&next))
    {
        return false;
    }

    *packet = next;
    return true;
}

size_t HhStatus_Encode(const HhStatus_t *status, uint8_t *out, size_t capacity)
{
    (void)status;
    (void)out;
    (void)capacity;
    return 0U;
}

bool HhStatus_Decode(const uint8_t *data, size_t length, HhStatus_t *status)
{
    (void)data;
    (void)length;
    (void)status;
    return false;
}