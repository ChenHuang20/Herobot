/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "Handle.h"

#include "BSP_DMA.h"
#include "Driver_DBUS.h"
#include "Driver_Chassis.h"
#include "Driver_ICM20600.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

extern TIM_HandleTypeDef htim6;

void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 stream0 global interrupt.
*/
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream1 global interrupt.
*/
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream3 global interrupt.
*/
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream5 global interrupt.
*/
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
* @brief This function handles CAN1 RX0 interrupts.
*/
void CAN1_RX0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan1);

	switch(hcan1.pRxMsg->StdId)
	{
		case 0x201:
			//左前轮
			_ChassisParam.Motor[0].RealSpeed = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
			_ChassisParam.Motor[0].RealCurrent = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);
			break;
		case 0x202:
			//右前轮
			_ChassisParam.Motor[1].RealSpeed = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
			_ChassisParam.Motor[1].RealCurrent = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);
			break;
		case 0x203:
			//左后轮
			_ChassisParam.Motor[2].RealSpeed = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
			_ChassisParam.Motor[2].RealCurrent = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);
			break;
		case 0x204:
			//右后轮
			_ChassisParam.Motor[3].RealSpeed = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
			_ChassisParam.Motor[3].RealCurrent = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);
			break;
		default :
			break;
	}
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);

}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	if((__HAL_UART_GET_FLAG(&Usart_Handle,UART_FLAG_IDLE)!=RESET))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&Usart_Handle);
		__HAL_DMA_DISABLE(Usart_Handle.hdmarx); 
		DMA_CLEAR_FLAG_ALL(Usart_Handle.hdmarx);
		if((BinSemaphoreDBUS != NULL))//接收到数据，并且二值信号量有效
		{
			xSemaphoreGiveFromISR(BinSemaphoreDBUS, &xHigherPriorityTaskWoken);	//释放二值信号量
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//如果需要的话进行一次任务切换
		}
		__HAL_DMA_SET_COUNTER(Usart_Handle.hdmarx,100);
		__HAL_DMA_ENABLE(Usart_Handle.hdmarx);
	}
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream7 global interrupt.
*/
//void DMA1_Stream7_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

//  /* USER CODE END DMA1_Stream7_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_uart5_tx);
//  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

//  /* USER CODE END DMA1_Stream7_IRQn 1 */
//}

/**
* @brief This function handles UART5 global interrupt.
*/
//void UART5_IRQHandler(void)
//{
//  /* USER CODE BEGIN UART5_IRQn 0 */

//  /* USER CODE END UART5_IRQn 0 */
//  HAL_UART_IRQHandler(&huart5);
//  /* USER CODE BEGIN UART5_IRQn 1 */

//  /* USER CODE END UART5_IRQn 1 */
//}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream1 global interrupt.
*/
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
* @brief This function handles CAN2 RX0 interrupts.
*/
//void CAN2_RX0_IRQHandler(void)
//{
//  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

//  /* USER CODE END CAN2_RX0_IRQn 0 */
//  HAL_CAN_IRQHandler(&hcan2);
//  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

//  /* USER CODE END CAN2_RX0_IRQn 1 */
//}

/**
* @brief This function handles DMA2 stream6 global interrupt.
*/
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream7 global interrupt.
*/
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/**
* @brief This function handles USART6 global interrupt.
*/


//void USART6_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART6_IRQn 0 */

//  /* USER CODE END USART6_IRQn 0 */
//  HAL_UART_IRQHandler(&huart6);
//  /* USER CODE BEGIN USART6_IRQn 1 */

//  /* USER CODE END USART6_IRQn 1 */
//}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

//void EXTI0_IRQHandler(void)
//{
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
//}

/**********************************************ICM20600中断任务****************************************************/
int uui = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uui++;
	if(uui % 2 == 0)
	icm20600_task();
}

void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
