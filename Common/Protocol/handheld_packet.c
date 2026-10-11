//
// Created by YZH on 2026/10/11.
//

#include "handheld_packet.h"
#define CRC8_POLY           0x07U
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
    (void)packet;
    (void)out;
    (void)capacity;
    return 0U;
}

bool HhPacket_Decode(const uint8_t *data, size_t length, HhPacket_t *packet)
{
    (void)data;
    (void)length;
    (void)packet;
    return false;
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