
#include "Handle.h"

#include "Driver_ICM20600.h"

#include "Task_IMU.h"

/**
  * @brief  IMU任务
  * @param  unused
  * @retval void
  */
void Task_IMU(void * pvParameters )
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	_StackSurplus.IMU = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		uint8_t EventIMU = 0;
		
		_Tick.IMU++;

		icm20600_task();
		
		EventIMU = 1;

		//设置监控IMU事件位BIT1
		if(EventGroupHandler!=NULL)
		{
			if(EventIMU == 1)
			{
				xEventGroupSetBits(EventGroupHandler,EVENTBIT_1);
			}
		}
		//获取剩余栈大小
		_StackSurplus.IMU = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelayUntil(&xLastWakeTime, (2 / portTICK_RATE_MS) );

    }
}
