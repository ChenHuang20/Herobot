#include "Handle.h"
#include <math.h>

#include "BSP_TIM.h"

#include "Driver_Stir.h"
#include "Driver_Friction.h"
#include "Driver_DBUS.h"
#include "Driver_Steering.h"
#include "Driver_Monitor.h"
#include "Driver_Judge.h"

#include "Algorithm_Pid.h"

#include "Task_Shoot.h"

#include "stm32f4xx_hal.h"
#define BULLET_SAVE 5

uint8_t shoot_last=0;
uint16_t test=0;
float Stir17_realPosition = 0.0f;

Shoot_t _shoot = { 0 };
uint32_t btick = 0;
float PreTargetSpeed = 0;
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
        

/******************************************小拨弹电机*************************************************/
		if(_radio.rc.mode == 1 && _shoot.friction17 == 0 && _radio.rc.shoot == 2)
		_shoot.continued_reverse17 = 1;
		else
		_shoot.continued_reverse17 = 0;
		if(_shoot.continued_reverse17 == 0)
		{
			if(_shoot.shooting_off17 == 1)
			{
				_shoot.reverse17 = 0;
				_Stir17Param.TargetCurrent = 0;
			}
			else
			{
				Shooting_Judge17();
				if(!_shoot.reverse17)
				{
					PID_Calc(&Stir17_position_pid, _Stir17Param.CurrentPosition, _Stir17Param.TargetPosition , POSITION_PID);
					_Stir17Param.TargetSpeed = Stir17_position_pid.output;
				}
			}
		}
		else
		{
			_Stir17Param.TargetSpeed = -0.9f;
			_Stir17Param.TargetPosition = _Stir17Param.CurrentPosition + _Stir17Param.Round_cnt;
		}
		PID_Calc(&Stir17_speed_pid, _Stir17Param.CurrentSpeed, _Stir17Param.TargetSpeed, DELTA_PID);
		_Stir17Param.TargetCurrent = (int16_t)(Stir17_speed_pid.output * 600.0f);		

/******************************************大拨弹电机*************************************************/

            _Stir42Param.Sensor = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15);


		if(_Stir42Param.Sensor == 0 && _Stir42Param.Sensor_last == 1){
			_Stir42Param.Bullet_count++;
			Stir42_ON();
		}
		if(_Stir42Param.Bullet_count == BULLET_SAVE) 	     //此处改变枪管堆积大弹丸数量（同时记录剩余大弹丸数）
			_Stir42Param.Stop_flag = 1;

		if(_Stir42Param.Stop_flag == 1){
		    _Stir42Param.Bullet_tick++;
			if(_Stir42Param.Bullet_tick > 10){                      //此处改变停转延迟时间
				 Stir42_OFF();
				_Stir42Param.Stop_flag = 0;
			}
		}
		_Stir42Param.Sensor_last = _Stir42Param.Sensor;

    if(_Stir42Param.Stop_flag == 1) {
            if(_Stir42Param.TargetSpeed > 0) {
                _Stir42Param.Stop_flag = 0;
            _Stir42Param.TargetSpeed = -0.18f;
                _Stir42Param.Blocked_tick = 0;

            }
            else {
            _Stir42Param.TargetSpeed = 0.09f;
                _Stir42Param.Stop_flag = 0;
                _Stir42Param.Blocked_tick = 0;
            }
        }

            PID_Calc(&Stir42_speed_pid, _Stir42Param.CurrentSpeed, _Stir42Param.TargetSpeed, DELTA_PID);
		  _Stir42Param.TargetCurrent = (int16_t)(Stir42_speed_pid.output * 300.0f);	
        

		if((BinSemaphoreShoot != NULL))//接收到数据，并且二值信号量有效
		{
			xSemaphoreGive(BinSemaphoreShoot);
		}
		//获取剩余栈大小
		_StackSurplus.Shoot = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelayUntil(&xLastWakeTime, (4 / portTICK_RATE_MS) );
	}
}

void Shooting_Judge17(void)
{
	if(fabs(Stir17_speed_pid.output) > 22.9f)//电流待调试
	{
		_shoot.blocking_tick17++;
		if(_shoot.blocking_tick17 == 230)
		{
			_shoot.reverse17 = 1;
			_shoot.reverse_count17++;
		}
		if(_shoot.blocking_tick17 >= 200)
		{
			_shoot.shooting_off17 = 1;
		}
	}
	else
		_shoot.blocking_tick17 = 0;
	if(_shoot.reverse17  == 1)
	{
		_Stir17Param.TargetSpeed = -0.15f;//速度待调试
		if(_shoot.reverse_count17 >= 2) {
			_shoot.shooting_off17 = 1;
		}
		_shoot.reverse_tick17++;
		if(_shoot.reverse_tick17 >= 50*_shoot.reverse_count17)
		{
			_shoot.reverse17 = 0;
			_Stir17Param.TargetPosition = _Stir17Param.CurrentPosition + _Stir17Param.Round_cnt;
		}
	}
	else
		_shoot.reverse_tick17 = 0;
		if(_shoot.arti_shoot17)
		{
			if(_shoot.reverse17 == 0)
			{
				if(_status.judgement == 0)//无裁判系统
				{
					_shoot.count_t17 = _shoot.count_c17 + 3;
					_shoot.dalay_ms17 = 500;
				}
				else
				{
					if(_Judge.robotLevel == 1)
					{
						_shoot.heatmax17 = 1600;
						_shoot.dalay_ms17 = 400;
						_shoot.shooting_speed17 = 0;
						_shoot.count_t17 = _shoot.count_c17+(_shoot.heatmax17/530 - 2);
					}
					else if(_Judge.robotLevel == 2)
					{
						_shoot.heatmax17 = 3000;
						_shoot.dalay_ms17 = 300;
						_shoot.shooting_speed17 = 0.7;
						_shoot.count_t17 = _shoot.count_c17+(_shoot.heatmax17/700 - 3);
					}
					else if(_Judge.robotLevel == 3)
					{
						_shoot.heatmax17 = 6000;
						_shoot.dalay_ms17 = 166;
						_shoot.shooting_speed17 = 1.5;
						_shoot.count_t17 = _shoot.count_c17+(_shoot.heatmax17/1000 - 3);
					}
				}
			}

			if(_shoot.friction17 == 1)
			{
				{
					if(_shoot.super_shoot_count17 > 0 && _shoot.shooting_tick17 + 60 <= Tick)
					{
						_shoot.super_shoot_count17--;
						_shoot.flag17 = 1;
					}
					else if(_shoot.count_c17 <= _shoot.count_t17 && _shoot.shooting_tick17+_shoot.dalay_ms17 <= Tick)
					{
						_shoot.count_c17++;
						_shoot.flag17 = 1;
					}
				}
			}
	}
	if(_shoot.flag17 == 1)
	{
		_Stir17Param.TargetPosition -= 6.0f;
		_shoot.shooting_tick17 = Tick;
		_shoot.flag17 = 0;
	}
//	if(_status.judgement == 1)
//	{
//		if(_shoot.heatmax < _Judge.shooterHeat0 + 1500)//超热量保险判断
//		_Stir17Param.TargetPosition = _Stir17Param.CurrentPosition + _Stir17Param.Round_cnt;
//	}
}
