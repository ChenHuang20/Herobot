
#include <math.h>
#include "Handle.h"

#include "Task_Gimbal.h"

#include "Driver_Gimbal.h"
#include "Driver_DBUS.h"

#include "Algorithm_Pid.h"
#include "Algorithm_Estimator.h"

/**
  * @brief  云台任务
  * @param  unused
  * @retval void
  */
void Task_Gimbal(void *Parameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	_StackSurplus.Gimbal = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
		_Tick.Gimbal++;

		//外环输出角度和角速度
		float euler[3] = { _attitude.euler[0], _attitude.euler[1], _attitude.euler[2] };
		float rates[3] = { _attitude.rates[0], _attitude.rates[1], _attitude.rates[2] };

		if (fabsf(_radio.rc.pitch) > 0.05f)
		{
			PitchParam.TargetAngle = euler[1];//pitch期望角度
			PitchParam.TargetRates = _radio.rc.pitch * 2.0f;//pitch期望角速度
		}
		else
		{
		    PID_Calc(&Pitch_position_pid, euler[1], PitchParam.TargetAngle, POSITION_PID);
			PitchParam.TargetRates = Pitch_position_pid.output;
		}

		if (fabsf(_radio.rc.yaw) > 0.05f)
		{
			YawParam.TargetAngle = euler[2];//yaw期望角度
			YawParam.TargetRates = _radio.rc.yaw * 2.0f;//yaw期望角速度
		}
		else
		{
			PID_Calc(&Yaw_position_pid, euler[2], YawParam.TargetAngle, POSITION_PID);
			YawParam.TargetRates = Yaw_position_pid.output;
		}
		PID_Calc(&Pitch_speed_pid, rates[1], PitchParam.TargetRates, POSITION_PID);
		PID_Calc(&Yaw_speed_pid, rates[2],YawParam.TargetRates, POSITION_PID);

		PitchParam.TargetCurrent = Pitch_speed_pid.output;
		YawParam.TargetCurrent = Yaw_speed_pid.output;

		if((BinSemaphoreGimbal != NULL))//接收到数据，并且二值信号量有效
		{
			xSemaphoreGive(BinSemaphoreGimbal);
		}
		//获取剩余栈大小
		_StackSurplus.Gimbal = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelayUntil(&xLastWakeTime, (2 / portTICK_RATE_MS) );
    }
}
