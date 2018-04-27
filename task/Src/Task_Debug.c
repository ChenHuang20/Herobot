
#include "Handle.h"
#include "Driver_Debug.h"
#include "Task_Debug.h"



/**
  * @brief  上位机调试任务
  * @param  unused
  * @retval void
  */
void Task_Debug(void *Parameters)
{

	_StackSurplus.Debug = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.Debug++;
		debug_task();
		//获取剩余栈大小
		_StackSurplus.Debug = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelay(100);
    }
}
