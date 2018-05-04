#include "Handle.h"
#include <math.h>

#include "Driver_Stir.h"
#include "Driver_Friction.h"
#include "Driver_DBUS.h"

#include "Algorithm_Pid.h"

#include "Task_Shoot.h"


uint8_t shoot_last=0;
uint16_t test=0;

/**
  * @brief  射击任务
  * @param  unused
  * @retval void
  */

void Task_Shoot(void *Parameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	_StackSurplus.Shoot = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.Shoot++;
									 
        //大拨弹速度环
		float Stir42_realSpeed = _Stir42Param.RealSpeed * 0.007958701389f / 19.0f;       
		_Stir42Param.TargetSpeed = 1.0f;
		PID_Calc(&Stir42_speed_pid, Stir42_realSpeed, _Stir42Param.TargetSpeed, DELTA_PID);
		_Stir42Param.TargetCurrent = (int16_t)(Stir42_speed_pid.output * 1000.0f);				

		//小拨弹电机位置环
		float Stir17_realSpeed = _Stir17Param.RealSpeed * 0.007958701389f / 36.0f;

		if((_Stir17Param.RealPosition - _Stir17Param.PrePositon) < -0.5*8192)       //过零点处理
			_Stir17Param.Round_cnt++;
		else if((_Stir17Param.RealPosition - _Stir17Param.PrePositon) > 0.5*8192)
			_Stir17Param.Round_cnt--;
		float Stir17_realPosition =	_Stir17Param.Round_cnt + _Stir17Param.RealPosition / 8192.0f;

		if(_radio.rc.shoot == 1 && shoot_last == 3 && _Stir17Param.Shoot_task == 0){	 //发射任务过程遥控器不再控制
			_Stir17Param.Shoot_task = 1;
			_Stir17Param.Reverse_task = 0;
			_Stir17Param.TargetPosition += 3.6f;
			_Stir17Param.Shoot_count = 1;
		}

		if(_Stir17Param.Shoot_task == 1 && _Stir17Param.Reverse_task == 0){	 //连拨模式
				if(_Stir17Param.Shoot_count <= _Stir17Param.Shoot_num)
				_Stir17Param.Shoot_tick++;
				if(_Stir17Param.Shoot_tick >= _Stir17Param.Shoot_time){
			    	if(_Stir17Param.Shoot_count == _Stir17Param.Shoot_num)
						_Stir17Param.Shoot_task = 0;
					else{
					_Stir17Param.TargetPosition += 3.6f;                		//一次转36度					
					_Stir17Param.Shoot_count++;	
					}
					_Stir17Param.Shoot_tick = 0;
				}
			}

		PID_Calc(&Stir17_position_pid, Stir17_realPosition,_Stir17Param.TargetPosition , POSITION_PID);
		_Stir17Param.TargetSpeed = Stir17_position_pid.output/5;
//		_Stir17Param.TargetSpeed = _radio.rc.x*3.6;
		PID_Calc(&Stir17_speed_pid, Stir17_realSpeed, _Stir17Param.TargetSpeed, DELTA_PID);
		_Stir17Param.TargetCurrent = Stir17_speed_pid.output*40;		
		_Stir17Param.PrePositon = _Stir17Param.RealPosition;

		if(_Stir17Param.TargetPosition - Stir17_realPosition > 1){                  
				_Stir17Param.Blocked_tick++;
				if(_Stir17Param.Blocked_tick > 100){
					_Stir17Param.Reverse_task = 1;  
					_Stir17Param.TargetPosition = _Stir17Param.TargetPosition-7.2f;		
					_Stir17Param.Blocked_tick = 0;
					_Stir17Param.Shoot_tick = 0;
					_Stir17Param.Shoot_count = 0;
				}
		}
		else if((Stir17_realPosition - _Stir17Param.TargetPosition) > 1 && (_Stir17Param.Reverse_task) == 1){
				_Stir17Param.Reverse_tick++;
				if(_Stir17Param.Reverse_tick > 200){
					_Stir17Param.TargetCurrent = 0;			
					_Stir17Param.Shoot_task = 0;       				//退出发射任务
				}
		}
		else if(fabs(_Stir17Param.TargetPosition - Stir17_realPosition) < 0.1){
				_Stir17Param.Blocked_tick = 0;
				_Stir17Param.Reverse_tick = 0;
			if(_Stir17Param.Reverse_task == 1){
				_Stir17Param.Reverse_task = 0;
				_Stir17Param.Shoot_task = 0;
			}
		}
		shoot_last = _radio.rc.shoot;

		if((BinSemaphoreShoot != NULL))//接收到数据，并且二值信号量有效
		{
			xSemaphoreGive(BinSemaphoreShoot);
		}
		//获取剩余栈大小
		_StackSurplus.Shoot = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS) );
	}
}
