//
// Created by YZH on 2026/10/11.
//

#ifndef XIAOSAI_NRF_LINK_H
#define XIAOSAI_NRF_LINK_H

#include <stdbool.h>
#include <stdint.h>

/* 手柄和主控共用这一份配置，任何一项不一致两边都收不到 */
#define NRF_LINK_CHANNEL        100U   // 2.500 GHz，避开 Wi-Fi；比赛时要和其他队错开
#define NRF_LINK_ADDR_WIDTH     5U
#define NRF_LINK_ADDR_INIT      {0x53U, 0x52U, 0x4DU, 0x32U, 0x37U}
#define NRF_LINK_MAX_PAYLOAD    32U
#define NRF_LINK_RETRY_COUNT    3U
#define NRF_LINK_RX_FIFO_DEPTH  3U
#define NRF_LINK_TX_TIMEOUT_MS  5U     // 重发间隔 500 us × 3 次，最坏约 2 ms，留余量

typedef enum
{
    NRF_TX_OK = 0,   // 对方已应答
    NRF_TX_LOST,     // 重发次数用完，对方没有应答
    NRF_TX_TIMEOUT   // 芯片没有反应，多半是 SPI 或供电问题
} NrfTxResult_t;

/**
 * @brief 按两端约定的参数配置模块，配置完成后 CE 保持高电平
 * @param mode nRF24_MODE_TX（手柄）或 nRF24_MODE_RX（主控）
 * @return 模块应答正常且 SPI 无故障返回 true
 * @note 模块上电后约 100 ms 才能访问，调用方负责等待
 */
bool NrfLink_Configure(uint8_t mode);

/**
 * @brief 发送一包并等待应答，仅供手柄调用
 * @param buf 待发送数据，长度 1～NRF_LINK_MAX_PAYLOAD
 * @param ack 输出应答里附带的载荷，至少 NRF_LINK_MAX_PAYLOAD 字节；传 NULL 表示丢弃
 * @param ack_len 输出应答载荷长度，没有载荷时为 0；ack 为 NULL 时可传 NULL
 * @return 发送结果；参数无效时返回 NRF_TX_TIMEOUT
 * @note 每次都会读空接收 FIFO，防止应答载荷堆满 FIFO 影响后续发送
 */
NrfTxResult_t NrfLink_Send(uint8_t *buf, uint8_t len, uint8_t *ack, uint8_t *ack_len);

/**
 * @brief 从接收 FIFO 取出一包，仅供主控调用
 * @param buf 输出缓冲区，至少 NRF_LINK_MAX_PAYLOAD 字节
 * @param len 输出实际长度
 * @return 取到有效数据返回 true；FIFO 为空或坏包返回 false
 */
bool NrfLink_Receive(uint8_t *buf, uint8_t *len);

/**
 * @brief 预装一包 ACK 载荷，随下一次应答发给手柄，仅供主控调用
 * @return 已写入返回 true；参数无效或 TX FIFO 里还有未发出的载荷时返回 false
 * @note 只在 TX FIFO 为空时写入，避免旧状态堆积，手柄收到的总是较新的状态
 */
bool NrfLink_QueueAck(const uint8_t *buf, uint8_t len);

#endif //XIAOSAI_NRF_LINK_H
