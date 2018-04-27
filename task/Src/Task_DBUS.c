
#include "Handle.h"

#include "Driver_DBUS.h"

#include "Task_DBUS.h"

/**
  * @brief  遥控器数据接收任务
  * @param  unused
  * @retval void
  */
void Task_DBUS(void *Parameters)
{
	BaseType_t Semaphore = pdFALSE;
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	_StackSurplus.DBUS = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.DBUS++;
		if(BinSemaphoreDBUS!=NULL)
		{
			Semaphore = xSemaphoreTake(BinSemaphoreDBUS,portMAX_DELAY);	//获取信号量
			if(Semaphore == pdTRUE)										//获取信号量成功
			{
				DBUS_DataProcessing();
				//获取剩余栈大小
				_StackSurplus.DBUS = uxTaskGetStackHighWaterMark(NULL);
			}
		}
		vTaskDelayUntil(&xLastWakeTime, (7 / portTICK_RATE_MS) );
    }
}
