#include "Driver_Stir.h"
#include "Algorithm_Pid.h"


Stir42Param_t _Stir42Param = {

    .TargetSpeed= 0,             
    .RealSpeed= 0,           
	.TargetCurrent= 0,          		
	.MaxStirSpeed= 2.0f,

	.kp = 4.0f,
	.ki = 0.05f,
	.kd = 0.0f,

 };

Stir17Param_t _Stir17Param = {

    .TargetSpeed = 0,             
    .RealSpeed = 0,  
	.PrePositon = 0,
	.TargetPosition = 0,
	.RealPosition = 0,
	.Round_cnt = 0,
	.TargetCurrent = 0,          		
	.MaxStirSpeed = 2.0f,
	.Blocked_tick = 0,
	.Reverse_task = 0,
	.Reverse_tick = 0,
	.Shoot_task = 0,
	.Shoot_num = 3,
	.Shoot_count = 0,
	.Shoot_lastcount = 0,
	.Shoot_time = 150,
	.Shoot_tick = 0,
	
	.kp_p = 1.5f,
	.ki_p = 0.001f,
	.kd_p = 0.0f,
	
	.kp_v = 200.5f,
	.ki_v = 0.0001f,
	.kd_v = 0.0f,
	

 };




void Stir_InitConfig(void)
{
	PID_Init(&Stir42_speed_pid,	  
			  _Stir42Param.kp,//kp
			  _Stir42Param.ki,//ki
			  _Stir42Param.kd,//kd
			  16.1f,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  3);			      //积分限幅
	
	PID_Init(&Stir17_speed_pid,	  
			  _Stir17Param.kp_v,//kp
			  _Stir17Param.ki_v,//ki
			  _Stir17Param.kd_v,//kd
			  1000.1f,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  3);			      //积分限幅	
	
	PID_Init(&Stir17_position_pid,	  
			  _Stir17Param.kp_p,//kp
			  _Stir17Param.ki_p,//ki
			  _Stir17Param.kd_p,//kd
			  1000.0f,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  0.01f);			      //积分限幅	
}


