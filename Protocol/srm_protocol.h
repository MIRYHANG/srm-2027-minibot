#ifndef SRM_PROTOCOL_H
#define SRM_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* “SRM校内赛”线协议的固定参数修改这些值会导致 App 与下位机不兼容 */
#define SRM_PROTOCOL_VERSION 4u
#define SRM_SYNC_1 0xA5u
#define SRM_SYNC_2 0x5Au
#define SRM_MAX_PAYLOAD 64u
#define SRM_MAX_FRAME (SRM_MAX_PAYLOAD + 6u)

/* VTYPE 字节低 4 位中的消息类型 */
typedef enum {
    SRM_TYPE_CONTROL = 0,
    SRM_TYPE_HELLO = 1,
    SRM_TYPE_DEBUG = 2,
    SRM_TYPE_ACK = 3,
    SRM_TYPE_ERROR = 4,
    SRM_TYPE_LOG = 5,
    SRM_TYPE_STATUS = 6,
    SRM_TYPE_PRO_CONTROL = 7
} srm_type_t;

/* 每向解析器送入一个字节后可能得到的结果负值表示当前候选帧无效 */
typedef enum {
    SRM_PARSE_NONE = 0,
    SRM_PARSE_FRAME = 1,
    SRM_PARSE_BAD_VERSION = -1,
    SRM_PARSE_BAD_LENGTH = -2,
    SRM_PARSE_BAD_CRC = -3
} srm_parse_result_t;

/*
 * CRC 校验通过后输出的帧视图
 * payload 指向解析器内部缓存，只能在下一次调用 srm_parser_push() 前使用；
 * 如果要放入任务队列异步处理，必须先复制 payload
 */
typedef struct {
    uint8_t version;
    uint8_t type;
    uint8_t sequence;
    uint8_t length;
    const uint8_t *payload;
} srm_frame_t;

/* CONTROL 消息解码后的完整遥控器状态四轴统一使用 -512..511 */
typedef struct {
    int16_t left_x;
    int16_t left_y;
    int16_t right_x;
    int16_t right_y;
    uint8_t buttons;  /* bit0=A, bit1=B, bit2=X, bit3=Y */
    uint8_t switches; /* bit0..bit5=S1..S6 */
    uint8_t dpad;     /* 0=center, 1=up, 2=down, 3=left, 4=right */
} srm_control_state_t;

/* PRO_CONTROL 的 24 位按键位图bit17..bit23 保留，收到非零值时整帧无效 */
#define SRM_PRO_BUTTON_A (1ul << 0)
#define SRM_PRO_BUTTON_B (1ul << 1)
#define SRM_PRO_BUTTON_X (1ul << 2)
#define SRM_PRO_BUTTON_Y (1ul << 3)
#define SRM_PRO_BUTTON_L1 (1ul << 4)
#define SRM_PRO_BUTTON_R1 (1ul << 5)
#define SRM_PRO_BUTTON_L2 (1ul << 6)
#define SRM_PRO_BUTTON_R2 (1ul << 7)
#define SRM_PRO_BUTTON_THUMB_L (1ul << 8)
#define SRM_PRO_BUTTON_THUMB_R (1ul << 9)
#define SRM_PRO_BUTTON_START (1ul << 10)
#define SRM_PRO_BUTTON_SELECT (1ul << 11)
#define SRM_PRO_BUTTON_MODE (1ul << 12)
#define SRM_PRO_BUTTON_DPAD_UP (1ul << 13)
#define SRM_PRO_BUTTON_DPAD_DOWN (1ul << 14)
#define SRM_PRO_BUTTON_DPAD_LEFT (1ul << 15)
#define SRM_PRO_BUTTON_DPAD_RIGHT (1ul << 16)
#define SRM_PRO_BUTTON_VALID_MASK 0x01FFFFul

/* 双摇杆为 -512..511，扳机保留完整的 0..255 HID 线性范围 */
typedef struct {
    int16_t left_x;
    int16_t left_y;
    int16_t right_x;
    int16_t right_y;
    uint8_t left_trigger;
    uint8_t right_trigger;
    uint32_t buttons;
} srm_pro_control_state_t;

/*
 * 流式解析器状态UART、SPP 和 BLE 都可能拆包或粘包，因此不要按一次接收
 * 对应一帧来处理，而应为收到的每个字节调用一次 srm_parser_push()
 */
typedef struct {
    uint8_t body[SRM_MAX_PAYLOAD + 4u];
    uint8_t state;
    uint8_t position;
    uint8_t expected;
} srm_parser_t;

/* 上电或重新连接后调用一次，清空收包状态 */
void srm_parser_init(srm_parser_t *parser);

/* 送入一个串口字节；返回 SRM_PARSE_FRAME 时 frame 中包含一帧完整消息 */
srm_parse_result_t srm_parser_push(srm_parser_t *parser, uint8_t byte, srm_frame_t *frame);

/* 计算协议使用的 CRC-8/ATM：多项式 0x07，初值 0x00 */
uint8_t srm_crc8_atm(const uint8_t *data, size_t length);

/*
 * 构建任意类型的完整发送帧成功返回 payload_length + 6，参数或容量无效返回 0
 */
size_t srm_build_frame(uint8_t *output, size_t capacity, uint8_t type, uint8_t sequence,
                       const uint8_t *payload, uint8_t payload_length);

/* 将控制状态编码成 CONTROL 帧，主要用于测试或下位机模拟发送 */
size_t srm_build_control(uint8_t *output, size_t capacity, uint8_t sequence,
                         const srm_control_state_t *state);

/* 校验并解码 CONTROL 帧；成功返回 1，类型、长度或字段范围错误返回 0 */
int srm_decode_control(const srm_frame_t *frame, srm_control_state_t *state);

/* PRO_CONTROL 成功返回 1HELLO 不是 CONTROL/PRO_CONTROL 的解码前置条件 */
size_t srm_build_pro_control(uint8_t *output, size_t capacity, uint8_t sequence,
                             const srm_pro_control_state_t *state);
int srm_decode_pro_control(const srm_frame_t *frame, srm_pro_control_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
