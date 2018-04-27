
#include "Task_Bell.h"
#include "cmsis_os.h"

void Task_Bell(void * pvParameters )
{
    while(1)
    {
		vTaskDelay(5);
    }
}

