
#include "Handle.h"

#include "Task_Gimbal.h"

/**
  * @brief  云台任务
  * @param  unused
  * @retval void
  */
void Task_Gimbal(void *Parameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	_StackSurplus.Gimbal = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.Gimbal++;
		//获取剩余栈大小
		_StackSurplus.Gimbal = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelayUntil(&xLastWakeTime, (2 / portTICK_RATE_MS) );
    }
}
