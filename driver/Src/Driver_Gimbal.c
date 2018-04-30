
#include "Driver_Gimbal.h"
#include "Driver_Chassis.h"

#include "Algorithm_Pid.h"

GimbalParam_t PitchParam = {

	//pitch速度
	.kp[0] = 4.0f,
	.ki[0] = 0.0f,
	.kd[0] = 0.0f,

	//pitch位置
	.kp[1] = 0.0f,
	.ki[1] = 0.0f,
	.kd[1] = 0.0f,
 };

GimbalParam_t YawParam = {

	//yaw速度
	.kp[0] = 4.0f,
	.ki[0] = 0.0f,
	.kd[0] = 0.0f,

	//yaw位置
	.kp[1] = 0.0f,
	.ki[1] = 0.0f,
	.kd[1] = 0.0f,
 };

/**
  * @brief  云台参数初始化
  * @param  void 
  * @retval void
  */
void Gimbal_InitConfig(void)
{
	PID_Init(&Pitch_speed_pid,	  //pitch电机速度环
			  PitchParam.kp[0],//kp
			  PitchParam.ki[0],//ki
			  PitchParam.kd[0],//kd
			  10000.1f,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  3);			      //积分限幅

	PID_Init(&Pitch_position_pid,//pitch电机位置环
			  PitchParam.kp[1],//kp
			  PitchParam.ki[1],//ki
			  PitchParam.kd[1],//kd
			  10000.1f,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  3);			      //积分限幅

	PID_Init(&Yaw_speed_pid,	 //yaw电机速度环
			  YawParam.kp[0],//kp
			  YawParam.ki[0],//ki
			  YawParam.kd[0],//kd
			  10000.1f,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  3);			      //积分限幅

	PID_Init(&Yaw_position_pid,	  //yaw电机位置环
			  YawParam.kp[1],//kp
			  YawParam.ki[1],//ki
			  YawParam.kd[1],//kd
			  10000.1f,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  3);			      //积分限幅
}
