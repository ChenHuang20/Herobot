
#include "Handle.h"
#include "Driver_PowerLimit.h"
#include "Driver_Chassis.h"
#include "Driver_Gimbal.h"
#include "Driver_DBUS.h"

#include "Task_Chassis.h"

#include "Algorithm_Pid.h"

#define RPM_TO_V 0.007958701389f
#define YAW_ABSANGLE 3000
#define PITCH_ABSANGLE 3000

/**
  * @brief  底盘任务
  * @param  unused
  * @retval void
  */
void Task_Chassis(void * pvParameters )
{
	_StackSurplus.Chassis = uxTaskGetStackHighWaterMark(NULL);

    while(1)
    {
		_Tick.Chassis ++;

		//max line speed is 4.06 m/s
		float MotoSpeed[4] = {_ChassisParam.Motor[0].RealSpeed * RPM_TO_V / 19.0f,
							  _ChassisParam.Motor[1].RealSpeed * RPM_TO_V / 19.0f,
							  _ChassisParam.Motor[2].RealSpeed * RPM_TO_V / 19.0f,
							  _ChassisParam.Motor[3].RealSpeed * RPM_TO_V / 19.0f};

		_ChassisParam.TargetVX  = _radio.rc.x * _ChassisParam.MaxWheelSpeed;
		_ChassisParam.TargetVY  = _radio.rc.y * _ChassisParam.MaxWheelSpeed;

		//无底盘跟随
		_ChassisParam.TargetOmega = _radio.rc.yaw * _ChassisParam.MaxWheelSpeed * 0.9f;

		//有底盘跟随
//		PID_Calc(&CM_rotate_pid, YawParam.RealEncoderAngle, YAW_ABSANGLE, POSITION_PID);
//		_ChassisParam.TargetOmega = CM_rotate_pid.output;

		_ChassisParam.Motor[0].TargetSpeed =  _ChassisParam.TargetVY + _ChassisParam.TargetVX + _ChassisParam.TargetOmega;
		_ChassisParam.Motor[1].TargetSpeed = -_ChassisParam.TargetVY + _ChassisParam.TargetVX + _ChassisParam.TargetOmega;
		_ChassisParam.Motor[2].TargetSpeed =  _ChassisParam.TargetVY - _ChassisParam.TargetVX + _ChassisParam.TargetOmega;
		_ChassisParam.Motor[3].TargetSpeed = -_ChassisParam.TargetVY - _ChassisParam.TargetVX + _ChassisParam.TargetOmega;

		float ChassisSpeed[4] = {_ChassisParam.Motor[0].TargetSpeed,_ChassisParam.Motor[1].TargetSpeed,_ChassisParam.Motor[2].TargetSpeed,_ChassisParam.Motor[3].TargetSpeed};

		//防止运动轨迹失真
//		Moving_Trial(ChassisSpeed);

		for(uint8_t i = 0; i < 4; i ++)
		{
		PID_Calc(&CM_speed_pid[i], MotoSpeed[i], ChassisSpeed[i], DELTA_PID);
		_ChassisParam.ChassisCurrent[i] = (int16_t)(CM_speed_pid[i].output * 1000.0f);
		}

//		if(PowerLimit.Cut==ON)
//		{
//			Current_Distribution(_ChassisParam.ChassisCurrent,8000);
//			if(PowerLimit.RemainPower[2]>=50) {
//				PowerLimit.Cut=OFF;
//			}
//		}
//		Current_Distribution(_ChassisParam.ChassisCurrent,10000);

	    if((BinSemaphoreChassis != NULL))//接收到数据，并且二值信号量有效
		{
			xSemaphoreGive(BinSemaphoreChassis);
		}
		//获取剩余栈大小
		_StackSurplus.Chassis = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelay(4);
    }
}


