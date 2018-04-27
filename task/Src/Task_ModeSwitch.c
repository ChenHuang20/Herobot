
#include "Handle.h"

#include "Task_ModeSwitch.h"


/**
  * @brief  ģʽѡ������
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
		//��ȡʣ��ջ��С
		_StackSurplus.ModeSwitch = uxTaskGetStackHighWaterMark(NULL);
		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS) );
    }
}
