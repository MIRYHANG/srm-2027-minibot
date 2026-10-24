//
// Created by YZH on 2026/10/11.
//

#include "nrf_link.h"

#include <stddef.h>

#include "NRF24.h"
#include "main.h"

bool NrfLink_Configure(uint8_t mode)
{
    const uint8_t addr[NRF_LINK_ADDR_WIDTH] = NRF_LINK_ADDR_INIT;

    NRF24_Port_ClearFault();
    nRF24_CE_L();
    nRF24_Init();

    // nRF24_Check 会把 TX 地址改写成 "nRF24"，必须先检查、再设置地址
    if (!nRF24_Check() || NRF24_Port_HasFault())
    {
        return false;
    }

    nRF24_SetRFChannel(NRF_LINK_CHANNEL);
    nRF24_SetDataRate(nRF24_DR_1Mbps);
    nRF24_SetCRCScheme(nRF24_CRC_2byte);
    nRF24_SetAddrWidth(NRF_LINK_ADDR_WIDTH);
    nRF24_SetAddr(nRF24_PIPETX, addr);
    nRF24_SetAddr(nRF24_PIPE0, addr);   // 发送端也用 pipe0 接收 ACK，地址必须和 TX 相同
    nRF24_SetRXPipe(nRF24_PIPE0, nRF24_AA_ON, NRF_LINK_MAX_PAYLOAD);
    nRF24_SetAutoRetr(nRF24_ARD_500us, NRF_LINK_RETRY_COUNT);
    nRF24_SetDynamicPayloadLength(nRF24_DPL_ON);
    nRF24_SetPayloadWithAck(1U);        // 下一阶段用 ACK 载荷回传车的状态
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);
    nRF24_SetOperationalMode(mode);
    nRF24_FlushTX();
    nRF24_FlushRX();
    nRF24_ClearIRQFlags();
    nRF24_SetPowerMode(nRF24_PWR_UP);

    // 发送端：FIFO 一有数据就自动发出，不需要 10 us 脉冲；接收端：保持接收
    nRF24_CE_H();

    return !NRF24_Port_HasFault();
}

/**
 * @brief 读空接收 FIFO，只保留最后一包应答载荷
 */
static void DrainAckPayloads(uint8_t *ack, uint8_t *ack_len)
{
    if (ack == NULL || ack_len == NULL)
    {
        nRF24_FlushRX();
        return;
    }

    // FIFO 最多 3 包，限制循环次数，SPI 异常时也不会卡在这里
    for (uint8_t count = 0U; count < NRF_LINK_RX_FIFO_DEPTH; count++)
    {
        if (nRF24_GetStatus_RXFIFO() == nRF24_STATUS_RXFIFO_EMPTY)
        {
            return;
        }

        uint8_t len = 0U;
        if (nRF24_ReadPayloadDpl(ack, &len) == nRF24_RX_EMPTY || NRF24_Port_HasFault())
        {
            *ack_len = 0U;
            return;
        }
        *ack_len = len;
    }
}

NrfTxResult_t NrfLink_Send(uint8_t *buf, uint8_t len, uint8_t *ack, uint8_t *ack_len)
{
    if (ack_len != NULL)
    {
        *ack_len = 0U;
    }

    if (buf == NULL || len == 0U || len > NRF_LINK_MAX_PAYLOAD)
    {
        return NRF_TX_TIMEOUT;
    }

    nRF24_WritePayload(buf, len);

    uint32_t start = HAL_GetTick();
    uint8_t flags;
    do
    {
        flags = nRF24_GetIRQFlags();
    } while ((flags & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) == 0U &&
             (uint32_t)(HAL_GetTick() - start) < NRF_LINK_TX_TIMEOUT_MS);

    nRF24_ClearIRQFlags();
    DrainAckPayloads(ack, ack_len);

    if ((flags & nRF24_FLAG_TX_DS) != 0U)
    {
        return NRF_TX_OK;
    }

    // 出现 MAX_RT 后不清空 TX FIFO，芯片不会再发下一包
    nRF24_FlushTX();
    return ((flags & nRF24_FLAG_MAX_RT) != 0U) ? NRF_TX_LOST : NRF_TX_TIMEOUT;
}

bool NrfLink_Receive(uint8_t *buf, uint8_t *len)
{
    if (buf == NULL || len == NULL)
    {
        return false;
    }

    if (nRF24_GetStatus_RXFIFO() == nRF24_STATUS_RXFIFO_EMPTY)
    {
        return false;
    }

    // 长度超过 32 的坏包，驱动会清空 FIFO 并把长度置 0
    nRF24_RXResult pipe = nRF24_ReadPayloadDpl(buf, len);
    nRF24_ClearIRQFlags();
    return pipe != nRF24_RX_EMPTY && *len > 0U;
}

bool NrfLink_IsConfigured(void)
{
    // FEATURE 复位值为 0，只有 bit0～2 有效；模块断开时 MISO 读到 0xFF，也不会误判
    uint8_t features = nRF24_GetFeatures();
    return !NRF24_Port_HasFault() &&
           features == (nRF24_FEATURE_EN_DPL | nRF24_FEATURE_EN_ACK_PAY);
}

bool NrfLink_QueueAck(const uint8_t *buf, uint8_t len)
{
    if (buf == NULL || len == 0U || len > NRF_LINK_MAX_PAYLOAD)
    {
        return false;
    }

    if (nRF24_GetStatus_TXFIFO() != nRF24_STATUS_TXFIFO_EMPTY)
    {
        return false;
    }

    // 驱动的接口参数是 char *，但不会修改数据
    nRF24_WriteAckPayload(nRF24_RX_PIPE0, (char *)buf, len);
    return true;
}
