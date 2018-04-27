
#include "Algorithm_Pid.h"
#include <math.h>

Pid_TypeDef CM_speed_pid[4] = { 0 };
Pid_TypeDef CM_rotate_pid = { 0 };
Pid_TypeDef Yaw_speed_pid = { 0 };
Pid_TypeDef Yaw_position_pid = { 0 };
Pid_TypeDef Pitch_speed_pid = { 0 };
Pid_TypeDef Pitch_position_pid = { 0 };
Pid_TypeDef Shoot_speed_pid = { 0 };
Pid_TypeDef Shoot_position_pid = { 0 };
Pid_TypeDef Power_Limit_pid = { 0 };

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
		if(fabsf(pid->err[NOW]) <= pid->intergral_band)//��ƫ��С�ڻ��ַ�Χ��ʱ�򣬽������
			pid->intergral += (pid->ki) * (pid->err[NOW]);//����ֵ
		else
			{pid->intergral = pid->intergral*0.99f;}//���ڻ��ַ�Χ�ڣ�˥�˻�������
		if(pid->intergral > pid->IntegralLimit)//��������
			pid->intergral = pid->IntegralLimit;
		else if(pid->intergral < -pid->IntegralLimit)//��������
			pid->intergral = -pid->IntegralLimit;

		pid->intergral += (pid->ki) * (pid->err[NOW]);//����ֵ
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



