
#include "Handle.h"

#include "Task_TakeBullet.h"

/**
  * @brief  ȡ������
  * @param  unused
  * @retval void
  */
void Task_TakeBullet(void *Parameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	_StackSurplus.TakeBullet = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.TakeBullet++;
		//��ȡʣ��ջ��С
		_StackSurplus.TakeBullet = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS) );
    }
}
