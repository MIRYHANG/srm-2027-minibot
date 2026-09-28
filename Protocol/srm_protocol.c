//
// Created by YZH on 2026/9/27
//

#include "srm_protocol.h"

#include <string.h>

/*
 * CRC-8/ATM 按最高位优先逐位计算校验范围由调用方决定；协议规定发送帧时
 * 从 VTYPE 算到 Payload 末尾，不包含 A5 5A 帧头，也不包含 CRC 字节本身
 */
uint8_t srm_crc8_atm(const uint8_t *data, size_t length) {
    uint8_t crc = 0u;
    size_t index;
    for (index = 0u; index < length; index++) {
        uint8_t bit;
        crc ^= data[index];
        for (bit = 0u; bit < 8u; bit++) {
            crc = (crc & 0x80u) != 0u ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

void srm_parser_init(srm_parser_t *parser) {
    if (parser == NULL) return;
    /* 清零后 state=0，解析器从“等待 0xA5”状态开始 */
    memset(parser, 0, sizeof(*parser));
}

/*
 * 当前候选帧出错后重新同步如果导致错误的字节本身恰好是 0xA5，就直接把它
 * 当作下一帧的第一个同步字节，避免漏掉紧随错误帧之后的有效帧
 */
static void reset_parser(srm_parser_t *parser, uint8_t byte) {
    parser->state = byte == SRM_SYNC_1 ? 1u : 0u;
    parser->position = 0u;
    parser->expected = 0u;
}

srm_parse_result_t srm_parser_push(srm_parser_t *parser, uint8_t byte, srm_frame_t *frame) {
    uint8_t length;
    uint8_t received_crc;
    if (parser == NULL || frame == NULL) return SRM_PARSE_BAD_LENGTH;

    /* 状态 0：忽略噪声，直到发现同步字节 0xA5 */
    if (parser->state == 0u) {
        if (byte == SRM_SYNC_1) parser->state = 1u;
        return SRM_PARSE_NONE;
    }
    /*
     * 状态 1：等待第二个同步字节 0x5A连续收到 0xA5 时继续停在本状态，
     * 这样 A5 A5 5A 也能从第二个 A5 正确开始解析
     */
    if (parser->state == 1u) {
        if (byte == SRM_SYNC_2) {
            parser->state = 2u;
            parser->position = 0u;
            parser->expected = 0u;
        } else if (byte != SRM_SYNC_1) {
            parser->state = 0u;
        }
        return SRM_PARSE_NONE;
    }

    /* 状态 2：帧头已经确认，依次缓存 VTYPE、SEQ、LEN、Payload 和 CRC */
    parser->body[parser->position++] = byte;

    /* v4 不接受其他线协议版本 */
    if (parser->position == 1u && (byte >> 4) != SRM_PROTOCOL_VERSION) {
        reset_parser(parser, byte);
        return SRM_PARSE_BAD_VERSION;
    }
    /* 收到 LEN 后才能计算本帧还需接收多少字节 */
    if (parser->position == 3u) {
        length = parser->body[2];
        if (length > SRM_MAX_PAYLOAD) {
            reset_parser(parser, byte);
            return SRM_PARSE_BAD_LENGTH;
        }
        parser->expected = (uint8_t)(3u + length + 1u);
    }
    /* 长度尚未知或整帧尚未收齐时，等待下一个串口字节 */
    if (parser->expected == 0u || parser->position < parser->expected) return SRM_PARSE_NONE;

    /* 收齐后先校验 CRC，失败的帧绝不能更新控制状态 */
    received_crc = parser->body[parser->expected - 1u];
    if (srm_crc8_atm(parser->body, parser->expected - 1u) != received_crc) {
        reset_parser(parser, byte);
        return SRM_PARSE_BAD_CRC;
    }
    /* CRC 正确：输出帧字段payload 是 body 内部的一段视图，不发生动态分配 */
    frame->version = parser->body[0] >> 4;
    frame->type = parser->body[0] & 0x0Fu;
    frame->sequence = parser->body[1];
    frame->length = parser->body[2];
    frame->payload = &parser->body[3];
    /* 当前帧已交付，立即回到等待下一帧头的状态 */
    parser->state = 0u;
    parser->position = 0u;
    parser->expected = 0u;
    return SRM_PARSE_FRAME;
}

size_t srm_build_frame(uint8_t *output, size_t capacity, uint8_t type, uint8_t sequence,
                       const uint8_t *payload, uint8_t payload_length) {
    size_t wire_length = (size_t)payload_length + 6u;
    /* 先检查所有边界，防止发送缓存越界或构造出协议不允许的帧 */
    if (output == NULL || capacity < wire_length || type > 0x0Fu
            || payload_length > SRM_MAX_PAYLOAD
            || (payload_length > 0u && payload == NULL)) return 0u;
    /* 按线格式依次写入帧头、VTYPE、序号、长度和 Payload */
    output[0] = SRM_SYNC_1;
    output[1] = SRM_SYNC_2;
    output[2] = (uint8_t)((SRM_PROTOCOL_VERSION << 4) | type);
    output[3] = sequence;
    output[4] = payload_length;
    if (payload_length > 0u) memcpy(&output[5], payload, payload_length);
    /* CRC 从 output[2] 的 VTYPE 开始计算，正好覆盖 VTYPE、SEQ、LEN 和 Payload */
    output[wire_length - 1u] = srm_crc8_atm(&output[2], (size_t)payload_length + 3u);
    return wire_length;
}

static void write_uint16_le(uint8_t *output, uint16_t value) {
    output[0] = (uint8_t)(value & 0xFFu);
    output[1] = (uint8_t)(value >> 8);
}

static uint16_t read_uint16_le(const uint8_t *input) {
    return (uint16_t)input[0] | ((uint16_t)input[1] << 8);
}

static int axis_in_range(int16_t value) {
    return value >= -512 && value <= 511;
}

static void pack_axes(uint8_t *output, int16_t left_x, int16_t left_y,
                      int16_t right_x, int16_t right_y) {
    uint16_t lx = (uint16_t)left_x & 0x03FFu;
    uint16_t ly = (uint16_t)left_y & 0x03FFu;
    uint16_t rx = (uint16_t)right_x & 0x03FFu;
    uint16_t ry = (uint16_t)right_y & 0x03FFu;
    output[0] = (uint8_t)lx;
    output[1] = (uint8_t)((lx >> 8) | (ly << 2));
    output[2] = (uint8_t)((ly >> 6) | (rx << 4));
    output[3] = (uint8_t)((rx >> 4) | (ry << 6));
    output[4] = (uint8_t)(ry >> 2);
}

static int16_t sign_extend_axis(uint16_t value) {
    value &= 0x03FFu;
    return value >= 0x0200u ? (int16_t)((int32_t)value - 0x0400) : (int16_t)value;
}

static void unpack_axes(const uint8_t *input, int16_t *left_x, int16_t *left_y,
                        int16_t *right_x, int16_t *right_y) {
    *left_x = sign_extend_axis((uint16_t)input[0]
            | ((uint16_t)(input[1] & 0x03u) << 8));
    *left_y = sign_extend_axis((uint16_t)(input[1] >> 2)
            | ((uint16_t)(input[2] & 0x0Fu) << 6));
    *right_x = sign_extend_axis((uint16_t)(input[2] >> 4)
            | ((uint16_t)(input[3] & 0x3Fu) << 4));
    *right_y = sign_extend_axis((uint16_t)(input[3] >> 6)
            | ((uint16_t)input[4] << 2));
}

size_t srm_build_control(uint8_t *output, size_t capacity, uint8_t sequence,
                         const srm_control_state_t *state) {
    uint8_t payload[7];
    uint16_t controls;
    if (state == NULL || !axis_in_range(state->left_x) || !axis_in_range(state->left_y)
            || !axis_in_range(state->right_x) || !axis_in_range(state->right_y)
            || state->dpad > 4u || (state->buttons & 0xF0u) != 0u
            || (state->switches & 0xC0u) != 0u) return 0u;
    pack_axes(payload, state->left_x, state->left_y, state->right_x, state->right_y);
    controls = (uint16_t)state->buttons
            | ((uint16_t)state->switches << 4)
            | ((uint16_t)state->dpad << 10);
    write_uint16_le(&payload[5], controls);
    return srm_build_frame(output, capacity, SRM_TYPE_CONTROL,
                           sequence, payload, (uint8_t)sizeof(payload));
}

int srm_decode_control(const srm_frame_t *frame, srm_control_state_t *state) {
    uint16_t controls;
    if (frame == NULL || state == NULL || frame->version != SRM_PROTOCOL_VERSION
            || frame->type != SRM_TYPE_CONTROL || frame->length != 7u) return 0;
    unpack_axes(frame->payload, &state->left_x, &state->left_y,
                &state->right_x, &state->right_y);
    controls = read_uint16_le(&frame->payload[5]);
    if ((controls & 0xE000u) != 0u) return 0;
    state->buttons = (uint8_t)(controls & 0x000Fu);
    state->switches = (uint8_t)((controls >> 4) & 0x003Fu);
    state->dpad = (uint8_t)((controls >> 10) & 0x0007u);
    /* 最后验证各字段范围；返回 0 时调用方必须保持原控制状态或进入安全状态 */
    return state->dpad <= 4u;
}

size_t srm_build_pro_control(uint8_t *output, size_t capacity, uint8_t sequence,
                             const srm_pro_control_state_t *state) {
    uint8_t payload[10];
    if (state == NULL || !axis_in_range(state->left_x) || !axis_in_range(state->left_y)
            || !axis_in_range(state->right_x) || !axis_in_range(state->right_y)
            || (state->buttons & ~SRM_PRO_BUTTON_VALID_MASK) != 0u) return 0u;
    pack_axes(payload, state->left_x, state->left_y, state->right_x, state->right_y);
    payload[5] = state->left_trigger;
    payload[6] = state->right_trigger;
    payload[7] = (uint8_t)(state->buttons & 0xFFu);
    payload[8] = (uint8_t)((state->buttons >> 8) & 0xFFu);
    payload[9] = (uint8_t)((state->buttons >> 16) & 0xFFu);
    return srm_build_frame(output, capacity, SRM_TYPE_PRO_CONTROL,
                           sequence, payload, (uint8_t)sizeof(payload));
}

int srm_decode_pro_control(const srm_frame_t *frame, srm_pro_control_state_t *state) {
    uint32_t buttons;
    if (frame == NULL || state == NULL || frame->version != SRM_PROTOCOL_VERSION
            || frame->type != SRM_TYPE_PRO_CONTROL || frame->length != 10u) return 0;
    unpack_axes(frame->payload, &state->left_x, &state->left_y,
                &state->right_x, &state->right_y);
    state->left_trigger = frame->payload[5];
    state->right_trigger = frame->payload[6];
    buttons = (uint32_t)frame->payload[7]
            | ((uint32_t)frame->payload[8] << 8)
            | ((uint32_t)frame->payload[9] << 16);
    if ((buttons & ~SRM_PRO_BUTTON_VALID_MASK) != 0u) return 0;
    state->buttons = buttons;
    return 1;
}
