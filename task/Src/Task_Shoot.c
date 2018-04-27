
#include "Handle.h"

#include "Task_Shoot.h"



/**
  * @brief  射击任务
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
		//获取剩余栈大小
		_StackSurplus.Shoot = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS) );
    }
}
