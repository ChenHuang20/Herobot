
#include "Handle.h"

#include "Driver_Judge.h"

#include "Task_Judge.h"

/**
  * @brief  裁判系统数据接收任务
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
			Semaphore = xSemaphoreTake(BinSemaphoreJudge,portMAX_DELAY);	//获取信号量
			if(Semaphore == pdTRUE)										//获取信号量成功
			{
				judge_read();

				Judge_SendData();

				_JudgeSend.SendAllow = 0;
				//获取剩余栈大小
				_StackSurplus.Judge = uxTaskGetStackHighWaterMark(NULL);
			}
		}
		vTaskDelayUntil(&xLastWakeTime, (20 / portTICK_RATE_MS) );
    }
}
