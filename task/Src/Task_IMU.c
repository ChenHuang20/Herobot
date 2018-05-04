
#include "Handle.h"

#include "Driver_ICM20600.h"

#include "Task_IMU.h"

/**
  * @brief  IMU����
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
		_Tick.IMU++;

		icm20600_task();

		//��ȡʣ��ջ��С
		_StackSurplus.IMU = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelayUntil(&xLastWakeTime, (2 / portTICK_RATE_MS) );
    }
}
