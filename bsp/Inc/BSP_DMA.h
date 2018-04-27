#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

void BSP_DMA_InitConfig(void);
extern void DMA_CLEAR_FLAG_ALL(DMA_HandleTypeDef *dmax);
extern void UART_DMA_RX_INIT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
extern void UART_DMA_TX_INIT(UART_HandleTypeDef* huart, uint32_t Size);
extern void Uart_Transmit_DMA(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);

#ifdef __cplusplus
}
#endif
