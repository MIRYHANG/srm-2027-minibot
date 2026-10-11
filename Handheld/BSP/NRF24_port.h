//
// Created by YZH on 2026/10/11.
//

#ifndef XIAOSAI_NRF_NRF24_PORT_H
#define XIAOSAI_NRF_NRF24_PORT_H

#include <stdbool.h>
#include <stdint.h>

void nRF24_CE_L(void);
void nRF24_CE_H(void);
void nRF24_CSN_L(void);
void nRF24_CSN_H(void);
uint8_t nRF24_LL_RW(uint8_t data);

bool NRF24_Port_HasFault(void);
void NRF24_Port_ClearFault(void);

#endif
