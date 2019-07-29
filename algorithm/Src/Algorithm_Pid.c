
#include "Algorithm_Pid.h"
#include <math.h>

Pid_TypeDef CM_speed_pid[4] = { 0 };
Pid_TypeDef CM_rotate_pid = { 0 };
Pid_TypeDef Yaw_speed_pid = { 0 };
Pid_TypeDef Yaw_position_pid = { 0 };
Pid_TypeDef Pitch_speed_pid = { 0 };
Pid_TypeDef Pitch_position_pid = { 0 };
Pid_TypeDef Stir42_speed_pid = { 0 };     
Pid_TypeDef Stir17_speed_pid = { 0 };  
Pid_TypeDef Stir17_position_pid = { 0 };
Pid_TypeDef Fric42_speed_pid[2] = { 0 };
Pid_TypeDef Fric17_speed_pid[2] = { 0 };
Pid_TypeDef Power_Limit_pid = { 0 };
Pid_TypeDef Yaw_speed_encode_pid = { 0 };
Pid_TypeDef Yaw_position_encode_pid = { 0 };
Pid_TypeDef Pitch_speed_encode_pid = { 0 };
Pid_TypeDef Pitch_position_encode_pid = { 0 };
Pid_TypeDef Stir42_position_pid = { 0 };
//��ʼ��PID
void PID_Init(Pid_TypeDef * pid,
			  float kp,
			  float ki,
			  float kd,
			  float max_out,
			  float dead_band,
			  float i_band,
			  float intergral_limit)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->maxout = max_out;
	pid->IntegralLimit = intergral_limit;
	pid->dead_band = dead_band;
	pid->intergral_band = i_band;

	pid->output = 0;

	pid->err[LLAST]=0;
	pid->err[LAST]=0;
	pid->err[NOW]=0;//2���� 1��һ�� 0���ϴ�

}

//PID����
void PID_Calc(Pid_TypeDef * pid,
			  float get,
			  float set,
			  uint8_t PID_mode)
{
	float p = 0,//�趨�����PIDֵ
		  i = 0,
		  d = 0;

	pid->err[NOW] = set - get;//��ǰ���

	//�Ƿ��������
	if(fabsf(pid->err[NOW]) >= pid->dead_band)//ƫ��ֵ���ڵ�������ֵ
	{
		pid->intergral += (pid->ki) * (pid->err[NOW]);//����ֵ

//		if(pid->intergral > pid->IntegralLimit)//��������
//			pid->intergral = pid->IntegralLimit;
//		else if(pid->intergral < -pid->IntegralLimit)//��������
//			pid->intergral = -pid->IntegralLimit;

		if(PID_mode == POSITION_PID)
		{
  		p = pid->kp * (pid->err[NOW]);
		i = pid->intergral;
		d = pid->kd * (pid->err[NOW] - pid->err[LAST]);

		pid->output = p + i + d;
		}
		else if(PID_mode == DELTA_PID)
		{
			p = pid->kp*((pid->err[NOW] - pid->err[LAST]));
			i = pid->ki*pid->err[NOW];
			d = pid->kd*(pid->err[NOW]-pid->err[LAST]*2+pid->err[LLAST]);
			pid->output += p + i + d;
		}
		//�Ƿ񳬳�������
		if(pid->output>pid->maxout)
			pid->output = pid->maxout;
		if(pid->output<-(pid->maxout))
			pid->output = -(pid->maxout);
	}
	else
		pid->output = 0;

	pid->err[LLAST] = pid->err[LAST];
	pid->err[LAST] = pid->err[NOW];
}



