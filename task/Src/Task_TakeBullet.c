
#include "Handle.h"

#include "Task_TakeBullet.h"

/**
  * @brief  取弹任务
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
		//获取剩余栈大小
		_StackSurplus.TakeBullet = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS) );
    }
}
