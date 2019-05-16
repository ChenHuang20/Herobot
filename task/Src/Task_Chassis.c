
#include "Handle.h"
#include "Driver_PowerLimit.h"
#include "Driver_Chassis.h"
#include "Driver_Gimbal.h"
#include "Driver_DBUS.h"
#include "Driver_ModeSwitch.h"

#include "Task_Chassis.h"
#include "Task_ModeSwitch.h"

#include "Algorithm_Pid.h"

#include "math.h"

//#define RPM_TO_V 0.007958701389f
//#define YAW_ABSANGLE 3000
//#define PITCH_ABSANGLE 3000

/**
  * @brief  底盘任务
  * @param  unused
  * @retval void
  */ 


float _Chassis_speed_gear[4] = { 0.5 , 0.7 , 0.85 , 1.0 };
float speed_rising_flag=1,speed_falling_flag=1;

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
//        if(single_press(_radio.key.A))
//        {
//            _radio.rc.x  -=0.004f;
//        }
//        if(single_press(_radio.key.S))
//        {
//            _radio.rc.y  -=0.004f;
//        }
//        if(single_press(_radio.key.D))
//        {
//            _radio.rc.x  +=0.004f;
//        }
//        if(single_press(_radio.key.Q))
//        {
//            _radio.rc.yaw  -=0.004f;
//        }
//        if(single_press(_radio.key.E))
//        {
//            _radio.rc.yaw  +=0.004f;
//        }
        //键鼠模式
        static float OriginVX = 0,OriginVY = 0,OriginVO =0;
        if(GlobalMode == PC_Ctrl_Mode)
        {
        OriginVY = ( OriginVY   >= 1    ? 1    : OriginVY + 0.02f) * (single_press(_radio.key.W) - single_press(_radio.key.S));
        OriginVX = ( OriginVX   >= 1    ? 1    : OriginVX + 0.02f) * (single_press(_radio.key.D) - single_press(_radio.key.A));
        OriginVO = ( OriginVO   >= 0.3f ? 0.3f : OriginVO + 0.01f) * (single_press(_radio.key.E) - single_press(_radio.key.Q));
        OriginVO += _radio.mouse.yaw * 0.2f;
        OriginVX += _radio.rc.x;
        OriginVY += _radio.rc.y;
        OriginVO += _radio.rc.yaw;
        }
        else if (GlobalMode == RC_Ctrl_Mode)
        {
            OriginVX = _radio.rc.x;
            OriginVY = _radio.rc.y;
            OriginVO = _radio.rc.yaw;
        }
		OriginVX *= _ChassisParam.MaxWheelSpeed * _Chassis_speed_gear[mode.Speed.Gear] * mode.WatchBack.status;
		OriginVY *= _ChassisParam.MaxWheelSpeed * _Chassis_speed_gear[mode.Speed.Gear] * mode.WatchBack.status;
        OriginVO *= _ChassisParam.MaxWheelSpeed;
        OriginVO -= mode.WatchBack.status == -1 ? 0.1 : 0;
        _ChassisParam.TargetVY = OriginVY*cosf((_GimbalParam[YAW].RealEncodeAngle - YAW_ABSANGLE)/8192*360) - OriginVX*sinf((_GimbalParam[YAW].RealEncodeAngle - YAW_ABSANGLE)/8192*360);
		_ChassisParam.TargetVX = OriginVY*sinf((_GimbalParam[YAW].RealEncodeAngle - YAW_ABSANGLE)/8192*360) + OriginVX*cosf((_GimbalParam[YAW].RealEncodeAngle - YAW_ABSANGLE)/8192*360);
        _ChassisParam.TargetOmega = OriginVO * 0.45f;
        _ChassisParam.TargetVX = OriginVX * 4;
        _ChassisParam.TargetVY = OriginVY * 4;
        //自由视角控制
        if(1 == mode.Freeview.status)
        {
            //无底盘跟随，开扭腰会跟随
            _ChassisParam.TargetOmega = _radio.rc.yaw * _ChassisParam.MaxWheelSpeed * 0.9f;
        }
        else
        {
            //有底盘跟随
            PID_Calc(&CM_rotate_pid, _GimbalParam[YAW].RealEncodeAngle, YAW_ABSANGLE, POSITION_PID);
            _ChassisParam.TargetOmega = CM_rotate_pid.output;
        }
        
        //扭腰
        Twist_Waist(mode.TwistWaist.status);

		_ChassisParam.Motor[0].TargetSpeed =  _ChassisParam.TargetVY + _ChassisParam.TargetVX + _ChassisParam.TargetOmega;
		_ChassisParam.Motor[1].TargetSpeed = -_ChassisParam.TargetVY + _ChassisParam.TargetVX + _ChassisParam.TargetOmega;
		_ChassisParam.Motor[2].TargetSpeed =  _ChassisParam.TargetVY - _ChassisParam.TargetVX + _ChassisParam.TargetOmega;
		_ChassisParam.Motor[3].TargetSpeed = -_ChassisParam.TargetVY - _ChassisParam.TargetVX + _ChassisParam.TargetOmega;

		float ChassisSpeed[4] = {_ChassisParam.Motor[0].TargetSpeed,
                                 _ChassisParam.Motor[1].TargetSpeed,
                                 _ChassisParam.Motor[2].TargetSpeed,
                                 _ChassisParam.Motor[3].TargetSpeed};

		//防止运动轨迹失真
//		Moving_Trial(ChassisSpeed);

		for(uint8_t i = 0; i < 4; i ++)
		{
		PID_Calc(&CM_speed_pid[i], MotoSpeed[i], ChassisSpeed[i], DELTA_PID);
		_ChassisParam.ChassisCurrent[i] = (int16_t)(CM_speed_pid[i].output * 1000.0f);
		}

		if(PowerLimit.Cut==ON)
		{
			Current_Distribution(_ChassisParam.ChassisCurrent,0);
			if(PowerLimit.RemainPower[2]>=50) {
				PowerLimit.Cut=OFF;
			}
		}
		Current_Distribution(_ChassisParam.ChassisCurrent,15000);

        if(GlobalMode == ShutDown)
        {
            for(uint8_t i = 0; i < 4; i ++)
            {
                _ChassisParam.ChassisCurrent[i] = 0 ;
            }
        }


	    if((BinSemaphoreChassis != NULL))//接收到数据，并且二值信号量有效
		{
			xSemaphoreGive(BinSemaphoreChassis);
		}
		//获取剩余栈大小
		_StackSurplus.Chassis = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelay(4);
    }
}


