
#include "Driver_Gimbal.h"
#include "Driver_Chassis.h"

#include "Algorithm_Pid.h"

GimbalParam_t PitchParam = {

	//pitch�ٶ�
	.kp[0] = 4.0f,
	.ki[0] = 0.0f,
	.kd[0] = 0.0f,

	//pitchλ��
	.kp[1] = 0.0f,
	.ki[1] = 0.0f,
	.kd[1] = 0.0f,
 };

GimbalParam_t YawParam = {

	//yaw�ٶ�
	.kp[0] = 4.0f,
	.ki[0] = 0.0f,
	.kd[0] = 0.0f,

	//yawλ��
	.kp[1] = 0.0f,
	.ki[1] = 0.0f,
	.kd[1] = 0.0f,
 };

/**
  * @brief  ��̨������ʼ��
  * @param  void 
  * @retval void
  */
void Gimbal_InitConfig(void)
{
	PID_Init(&Pitch_speed_pid,	  //pitch����ٶȻ�
			  PitchParam.kp[0],//kp
			  PitchParam.ki[0],//ki
			  PitchParam.kd[0],//kd
			  10000.1f,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  3);			      //�����޷�

	PID_Init(&Pitch_position_pid,//pitch���λ�û�
			  PitchParam.kp[1],//kp
			  PitchParam.ki[1],//ki
			  PitchParam.kd[1],//kd
			  10000.1f,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  3);			      //�����޷�

	PID_Init(&Yaw_speed_pid,	 //yaw����ٶȻ�
			  YawParam.kp[0],//kp
			  YawParam.ki[0],//ki
			  YawParam.kd[0],//kd
			  10000.1f,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  3);			      //�����޷�

	PID_Init(&Yaw_position_pid,	  //yaw���λ�û�
			  YawParam.kp[1],//kp
			  YawParam.ki[1],//ki
			  YawParam.kd[1],//kd
			  10000.1f,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  3);			      //�����޷�
}
