
#include "BSP_CAN.h"

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

/**
  * @brief  CAN³õÊ¼»¯
  * @param  void
  * @retval void
  */
void BSP_CAN_InitConfig(void)
{
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 3;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SJW = CAN_SJW_1TQ;
	hcan1.Init.BS1 = CAN_BS1_9TQ;
	hcan1.Init.BS2 = CAN_BS2_4TQ;
	hcan1.Init.TTCM = DISABLE;
	hcan1.Init.ABOM = DISABLE;
	hcan1.Init.AWUM = DISABLE;
	hcan1.Init.NART = DISABLE;
	hcan1.Init.RFLM = DISABLE;
	hcan1.Init.TXFP = ENABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 3;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SJW = CAN_SJW_1TQ;
	hcan2.Init.BS1 = CAN_BS1_9TQ;
	hcan2.Init.BS2 = CAN_BS2_4TQ;
	hcan2.Init.TTCM = DISABLE;
	hcan2.Init.ABOM = DISABLE;
	hcan2.Init.AWUM = DISABLE;
	hcan2.Init.NART = DISABLE;
	hcan2.Init.RFLM = DISABLE;
	hcan2.Init.TXFP = ENABLE;
	if (HAL_CAN_Init(&hcan2) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	can_filter_config(&hcan1);
	can_filter_config(&hcan2);
}

void can_filter_config(CAN_HandleTypeDef* _hcan)
{
    //can1 &can2 use same filter config
    CAN_FilterConfTypeDef  CAN_FilterConfigStructure;
    static CanTxMsgTypeDef Tx1Message;
    static CanRxMsgTypeDef Rx1Message;
    static CanTxMsgTypeDef Tx2Message;
    static CanRxMsgTypeDef Rx2Message;

    CAN_FilterConfigStructure.FilterNumber         = 0;
    CAN_FilterConfigStructure.FilterMode           = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale          = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh         = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow          = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh     = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow      = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.BankNumber           = 14; //can1(0-13)and can2(14-27)each get half filter
    CAN_FilterConfigStructure.FilterActivation     = ENABLE;

    if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK) {
    }
    //filter config for can2
    //can1(0-13),can2(14-27)
    CAN_FilterConfigStructure.FilterNumber = 14;
    if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK) {
    }

    if (_hcan == &hcan1) {
        _hcan->pTxMsg = &Tx1Message;
        _hcan->pRxMsg = &Rx1Message;
		HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
    }

    if (_hcan == &hcan2){
        _hcan->pTxMsg = &Tx2Message;
        _hcan->pRxMsg = &Rx2Message;
		HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
	}
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{

  
	if(hcan->Instance==CAN1)
	{
		/* Peripheral clock enable */
		HAL_RCC_CAN1_CLK_ENABLED++;
		if(HAL_RCC_CAN1_CLK_ENABLED==1){
			__HAL_RCC_CAN1_CLK_ENABLE();
	}

	
	}
	else if(hcan->Instance==CAN2)
	{
		/* Peripheral clock enable */
		__HAL_RCC_CAN2_CLK_ENABLE();
		HAL_RCC_CAN1_CLK_ENABLED++;
		if(HAL_RCC_CAN1_CLK_ENABLED==1){
			__HAL_RCC_CAN1_CLK_ENABLE();
		}

	}

}
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{

	if(hcan->Instance==CAN1)
	{
		//PA11     ------> CAN1_RX
		//PA12     ------> CAN1_TX
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
	}
	else if(hcan->Instance==CAN2)
	{
		__HAL_RCC_CAN2_CLK_DISABLE();
		//PB12     ------> CAN2_RX
		//PB13     ------> CAN2_TX
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
	}

}
