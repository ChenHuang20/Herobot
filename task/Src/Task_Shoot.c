
#include "Handle.h"

#include "Task_Shoot.h"



/**
  * @brief  �������
  * @param  unused
  * @retval void
  */
void Task_Shoot(void *Parameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	_StackSurplus.Shoot = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.Shoot++;
		//��ȡʣ��ջ��С
		_StackSurplus.Shoot = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS) );
    }
}
