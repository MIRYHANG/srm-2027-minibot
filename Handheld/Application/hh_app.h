//
// Created by YZH on 2026/10/22.
//

#ifndef XIAOSAI_NRF_HH_APP_H
#define XIAOSAI_NRF_HH_APP_H
#include <stdint.h>

void HhApp_Init(void);
void HhApp_Update(void);

/* 链路测试计数，在调试器 Live Watch 里观察 */
extern volatile uint32_t tx_ok_count;      // 对方已应答
extern volatile uint32_t tx_lost_count;    // 重发次数用完，对方没有应答
extern volatile uint32_t tx_timeout_count; // 芯片没有反应，多半是 SPI 或供电问题


#endif //XIAOSAI_NRF_HH_APP_H
