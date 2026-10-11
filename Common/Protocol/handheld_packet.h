//
// Created by YZH on 2026/10/11.
//

#ifndef XIAOSAI_HANDHELD_PACKET_H
#define XIAOSAI_HANDHELD_PACKET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * 手柄与主控之间的无线数据格式，两端共用这一份文件
 * 控制包：手柄 → 主控，每 10 ms 一包
 * 状态包：主控 → 手柄，放在 NRF24 的 ACK 载荷里随应答带回
 * 多字节字段一律小端（低字节在前），逐字节编码，不依赖结构体的内存布局
 * 修改任何字段的位置、大小或含义时，必须把 HH_PACKET_VERSION 加 1，两块板子都重新烧录
 */

/*---------------- 控制包：手柄 → 主控 ----------------*/
#define HH_PACKET_VERSION               1    // 协议版本，两端不一致时整包拒收
#define HH_PACKET_LEN                   15   // 协议版本1+序号1+前后左右4+使能急停标志位1+预设1+关节6+crc1
#define HH_AXIS_MAX                     1000 // 摇杆满量程
#define HH_JOG_MAX                      100  // 关节满量程
#define HH_JOINT_COUNT                  6    // 关节个数

/* 控制包 flags 字节的各位，1 表示“是” */
#define HH_FLAG_ENABLED                 0x01 // 0000 0001 使能，手柄端锁存：按使能键置 1，按失能键清 0
#define HH_FLAG_ESTOP                   0x02 // 0000 0010 失能键此刻正按着
#define HH_FLAG_ARM_STOP                0x04 // 0000 0100 机械臂停止，手柄端锁存：按停止键置 1，按使能键清 0
#define HH_FLAG_GEAR                    0x08 // 0000 1000 换挡键此刻正按着，按下沿和挡位由主控处理
#define HH_FLAG_TURN_CCW                0x10 // 0001 0000 逆时针自转键此刻正按着
#define HH_FLAG_TURN_CW                 0x20 // 0010 0000 顺时针自转键此刻正按着
#define HH_FLAG_VALID_MASK              0x3F // 0011 1111 上面六位的合集，其余位出现 1 时整包无效

/* 控制包 preset 字节的取值 */
#define HH_PRESET_NONE                  0    // 不切换预设，按关节点动
#define HH_PRESET_READY                 1    // 进入准备夹取位姿 READY
#define HH_PRESET_STOW                  2    // 回到收纳位姿 STOW

/*---------------- 状态包：主控 → 手柄 ----------------*/
#define HH_STATUS_LEN                   4    // 协议版本1+状态标志1+挡位1+crc1

/* 状态包 flags 字节的各位，1 表示“是” */
#define HH_STATUS_ENABLED               0x01 // 0000 0001 小车处于使能状态
#define HH_STATUS_SAFETY                0x02 // 0000 0010 车上急停按下或电机驱动故障
#define HH_STATUS_VALID_MASK            0x03 // 0000 0011 上面两位的合集，其余位出现 1 时整包无效

/* 控制包解码后的内容 */
typedef struct {
    uint8_t seq;                // 序号，每发一包加 1，255 之后回到 0，用于统计丢包
    int16_t forward;            // 前后，正数向前，-HH_AXIS_MAX～HH_AXIS_MAX
    int16_t left;               // 横移，正数向左，-HH_AXIS_MAX～HH_AXIS_MAX
    uint8_t flags;              // HH_FLAG_* 的组合
    uint8_t preset;             // HH_PRESET_* 之一
    int8_t jog[HH_JOINT_COUNT]; // 各关节点动，正数角度增大（夹爪为夹紧），-HH_JOG_MAX～HH_JOG_MAX
} HhPacket_t;

/* 状态包解码后的内容 */
typedef struct {
    uint8_t flags;              // HH_STATUS_* 的组合
    uint8_t gear;               // 小车当前挡位
} HhStatus_t;

/**
 * @brief 计算 CRC8校验码：多项式 0x07，初值 0x00
 * @param data 待校验的数据；为 NULL 时返回 0
 * @param length 数据字节数
 * @return 8 位校验码
 */
uint8_t HhPacket_Crc8(const uint8_t *data, size_t length);

/**
 * @brief 把控制包编码成 HH_PACKET_LEN 个字节，最后一个字节是 CRC
 * @param packet 待编码的控制包
 * @param out 输出缓冲区
 * @param capacity out 的容量，不能小于 HH_PACKET_LEN
 * @return 成功返回 HH_PACKET_LEN；参数为空、容量不足或任何字段越界返回 0，此时不能发送
 * @note 只写 out 的前 HH_PACKET_LEN 个字节
 */
size_t HhPacket_Encode(const HhPacket_t *packet, uint8_t *out, size_t capacity);

/**
 * @brief 校验并解码控制包
 * @param data 收到的字节
 * @param length 收到的字节数，必须等于 HH_PACKET_LEN
 * @param packet 输出解码结果
 * @return 长度、版本、CRC 和各字段范围全部正确返回 true
 *         任何一项不对返回 false，packet 保持调用前的内容不变
 */
bool HhPacket_Decode(const uint8_t *data, size_t length, HhPacket_t *packet);

/**
 * @brief 把状态包编码成 HH_STATUS_LEN 个字节，最后一个字节是 CRC
 * @param status 待编码的状态
 * @param out 输出缓冲区
 * @param capacity out 的容量，不能小于 HH_STATUS_LEN
 * @return 成功返回 HH_STATUS_LEN；参数为空、容量不足或标志含未定义的位返回 0
 */
size_t HhStatus_Encode(const HhStatus_t *status, uint8_t *out, size_t capacity);

/**
 * @brief 校验并解码状态包
 * @param data 收到的字节
 * @param length 收到的字节数，必须等于 HH_STATUS_LEN
 * @param status 输出解码结果
 * @return 长度、版本、CRC 和标志位全部正确返回 true
 *         任何一项不对返回 false，status 保持调用前的内容不变
 */
bool HhStatus_Decode(const uint8_t *data, size_t length, HhStatus_t *status);

#endif //XIAOSAI_HANDHELD_PACKET_H
