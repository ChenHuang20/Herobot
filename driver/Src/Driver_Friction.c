#include "BSP_TIM.h"
#include "Driver_Friction.h"
#include "Algorithm_Pid.h"
#include "Handle.h"

Fric42Param_t _Fric42Param = {

	.TargetSpeed[0] = 0,
	.TargetSpeed[1] = 0,

    .RealSpeed[0] = 0,
    .RealSpeed[1] = 0,

	.TargetCurrent[0] = 0,
	.TargetCurrent[1] = 0,

	.MaxStirSpeed[0] = 2.0f,
	.MaxStirSpeed[1] = 2.0f,

	.kp[0] = 4.0f,
	.kp[1] = 4.0f,	
	.ki[0] = 0.05f,
	.ki[1] = 0.05f,
	.kd[0] = 0.0f,
	.kd[1] = 0.0f,

};

void Fric_InitConfig(void){
	PID_Init(&Fric42_speed_pid[0],  
			  _Fric42Param.kp[0],
			  _Fric42Param.ki[0],
			  _Fric42Param.kd[0],
			  16.1f,
			  0,	  
			  0,	 
			  3);   

	PID_Init(&Fric42_speed_pid[1],
			  _Fric42Param.kp[1],
			  _Fric42Param.ki[1],
			  _Fric42Param.kd[1],
			  16.1f,
			  0,
			  0,	
			  3);     
}
 
void Fric42_Set(float TargetSpeed)
{
	_Fric42Param.TargetSpeed[0] = TargetSpeed;
	_Fric42Param.TargetSpeed[1] = TargetSpeed;

	//大摩擦轮速度环
	float Fric42_realSpeed[2] = {_Fric42Param.RealSpeed[0],      
								 _Fric42Param.RealSpeed[1]};

	PID_Calc(&Fric42_speed_pid[0], Fric42_realSpeed[0], _Fric42Param.TargetSpeed[0], DELTA_PID);
	PID_Calc(&Fric42_speed_pid[1], Fric42_realSpeed[1], _Fric42Param.TargetSpeed[1], DELTA_PID);

	_Fric42Param.TargetCurrent[0] = (int16_t)(Fric42_speed_pid[0].output * 1000.0f);
	_Fric42Param.TargetCurrent[1] = (int16_t)(Fric42_speed_pid[1].output * 1000.0f);
}

 
 void Fric17_ON()
{
	TIM2->CCR1 = 1000*0.40;
    TIM2->CCR2 = 1000*0.52;
}

void Fric17_OFF()
{
	TIM2->CCR1 = 1000*0.35;
    TIM2->CCR2 = 1000*0.35;
}
 
 