#include "BSP_TIM.h"
#include "Driver_Friction.h"
#include "Algorithm_Pid.h"
#include "Task_Shoot.h"
#include "Driver_Laser.h"
#include "Handle.h"
#include "BSP_TIM.h"

Fric42Param_t _Fric42Param = {

	.TargetSpeed[0] = 0,
	.TargetSpeed[1] = 0,

    .RealSpeed[0] = 0,
    .RealSpeed[1] = 0,

	.TargetCurrent[0] = 0,
	.TargetCurrent[1] = 0,

	.MaxStirSpeed[0] = 2.0f,
	.MaxStirSpeed[1] = 2.0f,

	.kp[0] = 10.0f,
	.kp[1] = 10.0f,	
	.ki[0] = 0.05f,
	.ki[1] = 0.05f,
	.kd[0] = 0.0f,
	.kd[1] = 0.0f,

};

void Fric_InitConfig(void){

    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
    
    Fric17_OFF();

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
void FricPID_InitConfig(void){

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
float pi[2],ou,yt;
void Fric42_SET(float TargetSpeed)
{
	_Fric42Param.TargetSpeed[0] = -TargetSpeed;
	_Fric42Param.TargetSpeed[1] = TargetSpeed;

	float Fric42_realSpeed[2] = {_Fric42Param.RealSpeed[0] / 7700.0f,      
								 _Fric42Param.RealSpeed[1] / 7700.0f};

                                 pi[0] = Fric42_realSpeed[0];\
                                  pi[1] = Fric42_realSpeed[1];
	PID_Calc(&Fric42_speed_pid[0], Fric42_realSpeed[0], _Fric42Param.TargetSpeed[0], DELTA_PID);
	PID_Calc(&Fric42_speed_pid[1], Fric42_realSpeed[1], _Fric42Param.TargetSpeed[1], DELTA_PID);

	_Fric42Param.TargetCurrent[0] = (int16_t)(Fric42_speed_pid[0].output * 330.0f);
	_Fric42Param.TargetCurrent[1] = (int16_t)(Fric42_speed_pid[1].output * 330.0f);

	_shoot.friction42 = 1;

	HAL_GPIO_WritePin(LASER_GPIO,LASER_GPIO_PIN,GPIO_PIN_SET);
}

void Fric42_OFF(void)
{
	_Fric42Param.TargetSpeed[0] = 0;
	_Fric42Param.TargetSpeed[1] = 0;

	float Fric42_realSpeed[2] = {_Fric42Param.RealSpeed[0],      
								 _Fric42Param.RealSpeed[1]};

	PID_Calc(&Fric42_speed_pid[0], Fric42_realSpeed[0], _Fric42Param.TargetSpeed[0], DELTA_PID);
	PID_Calc(&Fric42_speed_pid[1], Fric42_realSpeed[1], _Fric42Param.TargetSpeed[1], DELTA_PID);

	_Fric42Param.TargetCurrent[0] = 0;
	_Fric42Param.TargetCurrent[1] = 0;

	_shoot.friction42 = 0;
	
	HAL_GPIO_WritePin(LASER_GPIO,LASER_GPIO_PIN,GPIO_PIN_RESET);
}

void Fric17_SET(uint16_t TargetSpeed1,uint16_t TargetSpeed2)
{
	TIM2->CCR1 = TargetSpeed1;
    TIM2->CCR2 = TargetSpeed2;

	_shoot.friction17 = 1;

	HAL_GPIO_WritePin(LASER_GPIO,LASER_GPIO_PIN,GPIO_PIN_SET);
}

void Fric17_OFF(void)
{
	TIM2->CCR1 = 1000*0.55;
    TIM2->CCR2 = 1000*0.52;

	_shoot.friction17 = 0;

	HAL_GPIO_WritePin(LASER_GPIO,LASER_GPIO_PIN,GPIO_PIN_RESET);
}
