
#include "Handle.h"

#include "Task_ModeSwitch.h"


/**
  * @brief  模式选择任务
  * @param  unused
  * @retval void
  */
void Task_ModeSwitch(void *Parameters)
{

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	_StackSurplus.ModeSwitch = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.ModeSwitch++;
		//获取剩余栈大小
		_StackSurplus.ModeSwitch = uxTaskGetStackHighWaterMark(NULL);
		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS) );
    }
}
