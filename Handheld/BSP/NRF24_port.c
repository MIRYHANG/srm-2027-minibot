//
// Created by YZH on 2026/10/11.
//

#include "NRF24_port.h"
#include "main.h"
#include "spi.h"

#define NRF24_SPI_TIMEOUT_MS 2U

static volatile bool spi_fault = false;

void nRF24_CE_L(void)  { HAL_GPIO_WritePin(NRF_CE_GPIO_Port,  NRF_CE_Pin,  GPIO_PIN_RESET); }
void nRF24_CE_H(void)  { HAL_GPIO_WritePin(NRF_CE_GPIO_Port,  NRF_CE_Pin,  GPIO_PIN_SET); }
void nRF24_CSN_L(void) { HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET); }
void nRF24_CSN_H(void) { HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET); }

uint8_t nRF24_LL_RW(uint8_t data)
{
    uint8_t result = 0xFFU;

    if (HAL_SPI_TransmitReceive(&hspi1, &data, &result, 1U, NRF24_SPI_TIMEOUT_MS) != HAL_OK)
    {
        spi_fault = true;
        return 0xFFU;
    }
    return result;
}

bool NRF24_Port_HasFault(void)   { return spi_fault; }
void NRF24_Port_ClearFault(void) { spi_fault = false; }