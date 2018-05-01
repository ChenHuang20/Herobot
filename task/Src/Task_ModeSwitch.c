
#include "Handle.h"

#include "Driver_DBUS.h"
#include "Driver_ModeSwitch.h"

#include "Task_ModeSwitch.h"

/**
  * @brief  ģʽѡ������
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

		//ѡ�����ģʽ
		if(_radio.rc.mode == RC_UP)			 	//��
		{
			GlobalMode = RC_Ctrl_Mode;			//ң����ģʽ
		}
		else if(_radio.rc.mode == RC_MI)		//��
		{
			GlobalMode = PC_Ctrl_Mode;			//����ģʽ
		}
		else if(_radio.rc.mode == RC_DN)		//��
		{
			GlobalMode = ShutDown;				//�����ģʽ
		}
		switch(GlobalMode)
		{
			//ң����ģʽ
			case RC_Ctrl_Mode:

				if(_radio.rc.shoot == RC_UP)	//��
				{
					
				}
				if(_radio.rc.shoot == RC_MI)	//��
				{
					
				}
				if(_radio.rc.shoot == RC_DN)	//��
				{
					
				}

				break;

			//����ģʽ
			case PC_Ctrl_Mode:

/************************************************���̿���******************************************************/

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
/************************************************������******************************************************/

				if(_radio.mouse.press_l)	   //�������
				{
					
				}
				if(_radio.mouse.press_r)	   //�Ҽ�����
				{
					
				}
				break;

			//�����ģʽ
			case ShutDown:
				//output == 0;
				break;
			default:
				break;
		}
		//��ȡʣ��ջ��С
		_StackSurplus.ModeSwitch = uxTaskGetStackHighWaterMark(NULL);
		vTaskDelayUntil(&xLastWakeTime, (4 / portTICK_RATE_MS) );
    }
}
