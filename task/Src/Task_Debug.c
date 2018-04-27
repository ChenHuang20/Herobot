
#include "Handle.h"
#include "Driver_Debug.h"
#include "Task_Debug.h"



/**
  * @brief  ��λ����������
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
		//��ȡʣ��ջ��С
		_StackSurplus.Debug = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelay(100);
    }
}
