
#include "Handle.h"

#include "Driver_Judge.h"

#include "Task_Judge.h"

/**
  * @brief  ����ϵͳ���ݽ�������
  * @param  unused
  * @retval void
  */
void Task_Judge(void *Parameters)
{
	BaseType_t Semaphore = pdFALSE;
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	_StackSurplus.Judge = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.Judge++;
		if(BinSemaphoreJudge!=NULL)
		{
			Semaphore = xSemaphoreTake(BinSemaphoreJudge,portMAX_DELAY);	//��ȡ�ź���
			if(Semaphore == pdTRUE)										//��ȡ�ź����ɹ�
			{
				judge_read();

				Judge_SendData();

				_JudgeSend.SendAllow = 0;
				//��ȡʣ��ջ��С
				_StackSurplus.Judge = uxTaskGetStackHighWaterMark(NULL);
			}
		}
		vTaskDelayUntil(&xLastWakeTime, (20 / portTICK_RATE_MS) );
    }
}
