
#include "Handle.h"
#include "Driver_Monitor.h"

#include "Task_Monitor.h"


/**
  * @brief  �������
  * @param  unused
  * @retval void
  */
EventBits_t EventValue11;
void Task_Monitor(void *Parameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	static TickType_t xTicksToWait = 300 / portTICK_PERIOD_MS;//�����ʱ300ms
	EventBits_t EventValue;

	_StackSurplus.Monitor = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.Monitor++;

		EventValue = xEventGroupWaitBits((EventGroupHandle_t	)EventGroupHandler,//Ҫ����¼�����
										   (EventBits_t			)EVENTBIT_ALL,	   //��Ҫ����λ
										   (BaseType_t			)pdTRUE,		   //��Щλ�˳�ʱҪ�����	
										   (BaseType_t			)pdTRUE,		   //�ȴ���Щλ������λ
								           (TickType_t			)xTicksToWait);    //��ʱʱ��
		EventValue11 = EventValue;
		//�����¼�λ����λ
		if((EventValue & EVENTBIT_ALL) == EVENTBIT_ALL)
        {
            
        }
		//BIT_0�¼�δ����λ
		else if((EventValue & EVENTBIT_0) == 0)
		{
			
		}
		//BIT_1�¼�δ����λ
		else if((EventValue & EVENTBIT_1) == 0)
		{
			
		}
		//BIT_2�¼�δ����λ
		else if((EventValue & EVENTBIT_2) == 0)
		{
			
		}
		//BIT_3�¼�δ����λ
		else if((EventValue & EVENTBIT_3) == 0)
		{
			
		}
		//��ȡʣ��ջ��С
		_StackSurplus.Monitor = uxTaskGetStackHighWaterMark(NULL);
		
		if(_monitor.Judgement > 5)
			_status.judgement = 1;
		else
			_status.judgement = 0;

		//20ms���һ��
		vTaskDelayUntil(&xLastWakeTime, (20 / portTICK_RATE_MS) );
    }
}
