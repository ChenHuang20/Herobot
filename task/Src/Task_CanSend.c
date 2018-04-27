
#include "Handle.h"

#include "BSP_CAN.h"

#include "Driver_CanSend.h"
#include "Driver_Chassis.h"

#include "Task_CanSend.h"

#include "Algorithm_Pid.h"

/**
  * @brief  CAN��������
  * @param  unused
  * @retval void
  */
void Task_CanSend(void *Parameters)
{
	
	BaseType_t SemaphoreChassis = pdFALSE;
//	BaseType_t SemaphoreGimbal = pdFALSE;
	_StackSurplus.CanSend = uxTaskGetStackHighWaterMark(NULL);

    while(1)
    {
		HAL_StatusTypeDef EventCanChassis = HAL_TIMEOUT;
		_Tick.CanSend++;

		//��ȡ���������ź���
		if(BinSemaphoreChassis!=NULL)
		{
			SemaphoreChassis = xSemaphoreTake(BinSemaphoreChassis,portMAX_DELAY);

			if(SemaphoreChassis == pdTRUE)
			{
				EventCanChassis = send_cm_current(&hcan1, _ChassisParam.ChassisCurrent[0], _ChassisParam.ChassisCurrent[1], _ChassisParam.ChassisCurrent[2], _ChassisParam.ChassisCurrent[3]);
			}
		}
		//���ü�ص���CAN�¼�λBIT0
		if(EventGroupHandler!=NULL)
		{
			if(EventCanChassis == HAL_OK)
			{
				xEventGroupSetBits(EventGroupHandler,EVENTBIT_0);
			}
		}
		//��ȡʣ��ջ��С
		_StackSurplus.CanSend = uxTaskGetStackHighWaterMark(NULL);

	vTaskDelay(4);
    }

}
