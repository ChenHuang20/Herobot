
#include "BSP_DMA.h"

DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/**
  * @brief  DMA初始化
  * @param  void
  * @retval void
  */
void BSP_DMA_InitConfig(void)
{
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	//UART5_RX Init
    hdma_uart5_rx.Instance = DMA1_Stream0;
    hdma_uart5_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_rx.Init.Mode = DMA_NORMAL;
    hdma_uart5_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
    __HAL_LINKDMA(&huart5,hdmarx,hdma_uart5_rx);

    /* UART5_TX Init */
    hdma_uart5_tx.Instance = DMA1_Stream7;
    hdma_uart5_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart5_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart5_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_tx.Init.Mode = DMA_NORMAL;
    hdma_uart5_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart5_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart5_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
    __HAL_LINKDMA(&huart5,hdmatx,hdma_uart5_tx);

    //USART1_RX Init
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
    __HAL_LINKDMA(&huart1,hdmarx,hdma_usart1_rx);

    //USART1_TX Init
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
    __HAL_LINKDMA(&huart1,hdmatx,hdma_usart1_tx);

    //USART2_RX Init
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
    __HAL_LINKDMA(&huart2,hdmarx,hdma_usart2_rx);

    //USART3_RX Init
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
    __HAL_LINKDMA(&huart3,hdmarx,hdma_usart3_rx);

    //USART3_TX Init
    hdma_usart3_tx.Instance = DMA1_Stream3;
    hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
    __HAL_LINKDMA(&huart3,hdmatx,hdma_usart3_tx);
    //USART6_RX Init
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
    __HAL_LINKDMA(&huart6,hdmarx,hdma_usart6_rx);
    //USART6_TX Init
    hdma_usart6_tx.Instance = DMA2_Stream6;
    hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_tx.Init.Mode = DMA_NORMAL;
    hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
    __HAL_LINKDMA(&huart6,hdmatx,hdma_usart6_tx);
}

void DMA_CLEAR_FLAG_ALL(DMA_HandleTypeDef *dmax)
{
    uint32_t ele=(uint32_t)dmax->Instance;

    (ele == (uint32_t)DMA1_Stream0)? (DMA1->LIFCR=0x0000003D) :\
    (ele == (uint32_t)DMA1_Stream1)? (DMA1->LIFCR=0x00000F40) :\
    (ele == (uint32_t)DMA1_Stream2)? (DMA1->LIFCR=0x003D0000) :\
    (ele == (uint32_t)DMA1_Stream3)? (DMA1->LIFCR=0x0F400000) :\
    (ele == (uint32_t)DMA1_Stream4)? (DMA1->HIFCR=0x0000003D) :\
    (ele == (uint32_t)DMA1_Stream5)? (DMA1->HIFCR=0x00000F40) :\
    (ele == (uint32_t)DMA1_Stream6)? (DMA1->HIFCR=0x003D0000) :\
    (ele == (uint32_t)DMA1_Stream7)? (DMA1->HIFCR=0x0F400000) :\
    (ele == (uint32_t)DMA2_Stream0)? (DMA2->LIFCR=0x0000003D) :\
    (ele == (uint32_t)DMA2_Stream1)? (DMA2->LIFCR=0x00000F40) :\
    (ele == (uint32_t)DMA2_Stream2)? (DMA2->LIFCR=0x003D0000) :\
    (ele == (uint32_t)DMA2_Stream3)? (DMA2->LIFCR=0x0F400000) :\
    (ele == (uint32_t)DMA2_Stream4)? (DMA2->HIFCR=0x0000003D) :\
    (ele == (uint32_t)DMA2_Stream5)? (DMA2->HIFCR=0x00000F40) :\
    (ele == (uint32_t)DMA2_Stream6)? (DMA2->HIFCR=0x003D0000) :\
    (DMA2->HIFCR=0x0F400000);
}

void UART_DMA_RX_INIT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
    //使能串口DMA接收
    __HAL_DMA_DISABLE(huart->hdmarx);
    huart->hdmarx->Instance->PAR = (uint32_t)&huart->Instance->DR;//PAR is DMA stream x peripheral address register
    huart->hdmarx->Instance->NDTR = Size;
    huart->hdmarx->Instance->M0AR = (uint32_t)pData;
    DMA_CLEAR_FLAG_ALL(huart->hdmarx);
    __HAL_DMA_ENABLE(huart->hdmarx);
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    //开启串口空闲中断
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
}

void UART_DMA_TX_INIT(UART_HandleTypeDef* huart, uint32_t Size)
{
    __HAL_DMA_DISABLE(huart->hdmatx);
	huart->hdmatx->Instance->PAR = (uint32_t)&huart->Instance->DR;
	huart->hdmatx->Instance->NDTR = Size;
	huart->hdmatx->Instance->M0AR = NULL;
	DMA_CLEAR_FLAG_ALL(huart->hdmatx);
    //开启DMA发送中断
	__HAL_DMA_ENABLE_IT(huart->hdmatx,DMA_IT_TC);
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);
}

void Uart_Transmit_DMA(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
	__HAL_DMA_DISABLE(huart->hdmatx);
	huart->hdmatx->Instance->NDTR = Size;
	huart->hdmatx->Instance->M0AR = (uint32_t)pData;
	DMA_CLEAR_FLAG_ALL(huart->hdmatx);
	__HAL_DMA_ENABLE(huart->hdmatx);

}
