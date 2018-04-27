
#include "Task_Control.h"
#include "cmsis_os.h"

void Task_Control(void * pvParameters )
{
	portTickType xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS) );
    }
}

