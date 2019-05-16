
#include "Handle.h"
#include "Driver_Monitor.h"

#include "Task_Monitor.h"


/**
  * @brief  监控任务
  * @param  unused
  * @retval void
  */
EventBits_t EventValue11;
void Task_Monitor(void *Parameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	static TickType_t xTicksToWait = 300 / portTICK_PERIOD_MS;//最大延时300ms
	EventBits_t EventValue;

	_StackSurplus.Monitor = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.Monitor++;

		EventValue = xEventGroupWaitBits((EventGroupHandle_t	)EventGroupHandler,//要检测事件组句柄
										   (EventBits_t			)EVENTBIT_ALL,	   //需要检测的位
										   (BaseType_t			)pdTRUE,		   //这些位退出时要被清楚	
										   (BaseType_t			)pdTRUE,		   //等待这些位均被置位
								           (TickType_t			)xTicksToWait);    //超时时间
		EventValue11 = EventValue;
		//所有事件位均置位
		if((EventValue & EVENTBIT_ALL) == EVENTBIT_ALL)
        {
            
        }
		//BIT_0事件未被置位
		else if((EventValue & EVENTBIT_0) == 0)
		{
			
		}
		//BIT_1事件未被置位
		else if((EventValue & EVENTBIT_1) == 0)
		{
			
		}
		//BIT_2事件未被置位
		else if((EventValue & EVENTBIT_2) == 0)
		{
			
		}
		//BIT_3事件未被置位
		else if((EventValue & EVENTBIT_3) == 0)
		{
			
		}
		//获取剩余栈大小
		_StackSurplus.Monitor = uxTaskGetStackHighWaterMark(NULL);
		
		if(_monitor.Judgement > 5)
			_status.judgement = 1;
		else
			_status.judgement = 0;

		//20ms检测一次
		vTaskDelayUntil(&xLastWakeTime, (20 / portTICK_RATE_MS) );
    }
}
