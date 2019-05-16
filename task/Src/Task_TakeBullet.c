
#include "Handle.h"

#include "Driver_SynchroBelt.h"
#include "Driver_CanSend.h"
#include "Task_TakeBullet.h"
#include "Driver_DBUS.h"
#include "Driver_ModeSwitch.h"
#include "BSP_TIM.h"
/**
  * @brief  取弹任务
  * @param  unused
  * @retval void
  */
uint16_t EnterTick = 0, TickRecode;
 uint8_t flag=0;
long SynchroBeltTempPosition = 0, ArmTempPosition = 0;
void Task_TakeBullet(void *Parameters)
{

	_StackSurplus.TakeBullet = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.TakeBullet++;

   if(mode.WatchBack.status == -1) {
            TIM4->CCR4=780;
        }
        else {
              TIM4->CCR4=460;
        }

		//获取剩余栈大小
		_StackSurplus.TakeBullet = uxTaskGetStackHighWaterMark(NULL);
    	vTaskDelay(4);
    }

}
