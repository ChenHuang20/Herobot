
#include "Handle.h"

#include "Driver_DBUS.h"
#include "Driver_ModeSwitch.h"

#include "Task_ModeSwitch.h"

/**
  * @brief  模式选择任务
  * @param  unused
  * @retval void
  */
void Task_ModeSwitch(void *Parameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	_StackSurplus.ModeSwitch = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.ModeSwitch++;

		//选择操作模式
		if(_radio.rc.mode == RC_UP)			 	//上
		{
			GlobalMode = RC_Ctrl_Mode;			//遥控器模式
		}
		else if(_radio.rc.mode == RC_MI)		//中
		{
			GlobalMode = PC_Ctrl_Mode;			//键鼠模式
		}
		else if(_radio.rc.mode == RC_DN)		//下
		{
			GlobalMode = ShutDown;				//零输出模式
		}
		switch(GlobalMode)
		{
			//遥控器模式
			case RC_Ctrl_Mode:

				if(_radio.rc.shoot == RC_UP)	//上
				{
					
				}
				if(_radio.rc.shoot == RC_MI)	//中
				{
					
				}
				if(_radio.rc.shoot == RC_DN)	//下
				{
					
				}

				break;

			//键鼠模式
			case PC_Ctrl_Mode:

/************************************************键盘控制******************************************************/

				if(single_press(_radio.key.W))
				{
					
				}
				if(single_press(_radio.key.A))
				{
					
				}
				if(single_press(_radio.key.S))
				{
					
				}
				if(single_press(_radio.key.D))
				{
					
				}
				if(single_press(_radio.key.Q))
				{
					
				}
				if(single_press(_radio.key.E))
				{
					
				}
				if(single_press(_radio.key.Shift))
				{
					
				}
				if(single_press(_radio.key.Ctrl))
				{
					
				}
/************************************************鼠标控制******************************************************/

				if(_radio.mouse.press_l)	   //左键按下
				{
					
				}
				if(_radio.mouse.press_r)	   //右键按下
				{
					
				}
				break;

			//零输出模式
			case ShutDown:
				//output == 0;
				break;
			default:
				break;
		}
		//获取剩余栈大小
		_StackSurplus.ModeSwitch = uxTaskGetStackHighWaterMark(NULL);
		vTaskDelayUntil(&xLastWakeTime, (4 / portTICK_RATE_MS) );
    }
}
