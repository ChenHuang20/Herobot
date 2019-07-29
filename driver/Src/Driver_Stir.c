#include "Driver_Stir.h"
#include "Algorithm_Pid.h"
#include "BSP_TIM.h"


Stir42Param_t _Stir42Param = {

    .TargetSpeed = 0.08f,             
    .RealSpeed = 0,           
	.TargetCurrent = 0,          		
	.MaxStirSpeed = 2.0f,

    .LastPosition = 0,
    .PrePositon = 0,
    .PreBlockPositon = 10000,
    .EncoderPosition = 0,
	.Round_cnt = 0,
	.msg_cnt = 0,
    .CurrentPosition = 0,

	.Sensor = 0,              
	.Sensor_last = 0,   	         
	.Bullet_count = 0,		 
	.Stop_flag = 0,          
	.Bullet_tick = 0,         
	.Shoot_count = 0,
	.Blocked_tick = 0,

//	.kp_p = 5.5f,
//	.ki_p = 0.015f,
//	.kd_p = 30.0f,

	.kp_v = 60.0f,
	.ki_v = 0.39f,
	.kd_v = 0.0f,
 };

Stir17Param_t _Stir17Param = {

    .TargetSpeed = 0,             
    .RealSpeed = 0,  
    .CurrentSpeed = 0,
	.PrePositon = 0,
	.TargetPosition = 0,
	.RealPosition = 0,
    .EncoderPosition = 0,
    .CurrentPosition = 0,
	.Round_cnt = 0,
	.TargetCurrent = 0,          		
	.MaxStirSpeed = 2.0f,
	.Blocked_tick = 0,          //��¼����ʱ��
	.Reverse_task = 0,			//��ת��־λ
	.Reverse_tick = 0,			//��¼��תʱ��
	.Shoot_task = 0,			//���������־λ
	.Shoot_num = 5,				//�����ӵ���***************����
	.Shoot_count = 0,			//��¼һ�������ӵ�����
	.Shoot_time = 60,			//������ʱ��*************time=Shoot_time*�����������ʱ��
	.Shoot_tick = 0,			//��¼����ʱ��

	.kp_p = 5.5f,
	.ki_p = 0.015f,
	.kd_p = 30.0f,

	.kp_v = 1.2f,
	.ki_v = 0.0f,
	.kd_v = 0.0f,

};


void Stir_InitConfig(void)
{
	PID_Init(&Stir42_speed_pid,	  
			  _Stir42Param.kp_v,//kp
			  _Stir42Param.ki_v,//ki
			  _Stir42Param.kd_v,//kd
			  15.9f,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  4);			      //�����޷�	

//	PID_Init(&Stir42_position_pid,	  
//			  _Stir42Param.kp_p,//kp
//			  _Stir42Param.ki_p,//ki
//			  _Stir42Param.kd_p,//kd
//			  1000.0f,			  //PID������
//			  0,				  //����
//			  0,				  //���ַ�Χ
//			  0.01f);			  //�����޷�

	PID_Init(&Stir17_speed_pid,	  
			  _Stir17Param.kp_v,//kp
			  _Stir17Param.ki_v,//ki
			  _Stir17Param.kd_v,//kd
			  21.9f,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  3);			      //�����޷�	

	PID_Init(&Stir17_position_pid,	  
			  _Stir17Param.kp_p,//kp
			  _Stir17Param.ki_p,//ki
			  _Stir17Param.kd_p,//kd
			  16.0f,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  0.01f);			      //�����޷�	
}

void Stir42_ON(void)
{
	_Stir42Param.TargetSpeed = 0.08f;
}

void Stir42_OFF(void)
{
	_Stir42Param.TargetSpeed = 0;
}

void Stir17_ON(void)
{
	TIM2->CCR1 = 700;
    TIM2->CCR2 = 700;
}

void Stir17_OFF(void)
{
	TIM2->CCR1 = 520;
    TIM2->CCR2 = 520;
}
