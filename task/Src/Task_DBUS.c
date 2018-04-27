
#include "Handle.h"

#include "Driver_DBUS.h"

#include "Task_DBUS.h"

/**
  * @brief  ң�������ݽ�������
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
			Semaphore = xSemaphoreTake(BinSemaphoreDBUS,portMAX_DELAY);	//��ȡ�ź���
			if(Semaphore == pdTRUE)										//��ȡ�ź����ɹ�
			{
				DBUS_DataProcessing();
				//��ȡʣ��ջ��С
				_StackSurplus.DBUS = uxTaskGetStackHighWaterMark(NULL);
			}
		}
		vTaskDelayUntil(&xLastWakeTime, (7 / portTICK_RATE_MS) );
    }
}
