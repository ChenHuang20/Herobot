#include "Driver_Gimbal.h"

#include "Algorithm_Pid.h"

GimbalParam_t _GimbalParam[2] = {
  {                                   //YAW
    .RealEncodeAngle = 0,             //ʵ�ʱ�����������е�Ƕ�
    .MiddleEncoderAngle = 0,          //�м�λ�õĻ�е�Ƕ�
    .FrameCounter = 0,                //֡�ʼ�����
    .FrameRate = 0,                   //֡��
    .SendCurrent = 0,                //CAN���ͱ����еĵ���ֵ
	.kp[EULER] = 1.0f,//5.0
    .ki[EULER] = 0.0f,
    .kd[EULER] = 0.0f,

	.kp[RATES] = 1.0f,//1.2
    .ki[RATES] = 0.0f,
    .kd[RATES] = 0.0f,

	.kp[ANGLE] = 0.0f,
	.kp[SPEED] = 0.0f,//1.0
	.ki[ANGLE] = 0.0f,
	.ki[SPEED] = 0.0f,
	.kd[ANGLE] = 0.0f,
	.kd[SPEED] = 0.0f,
  },
  {                                   //PITCH
    .RealEncodeAngle = 0,             //ʵ�ʱ�����������е�Ƕ�
    .MiddleEncoderAngle = 0,          //�м�λ�õĻ�е�Ƕ�
    .FrameCounter = 0,                //֡�ʼ�����
    .FrameRate = 0,                   //֡��
    .SendCurrent = 0,                //CAN���ͱ����еĵ���ֵ
	.kp[EULER] = 1.0f,//10.0
    .ki[EULER] = 0.0f,
    .kd[EULER] = 0.0f,  

	.kp[RATES] = 0.6f,//0.4
    .ki[RATES] = 0.0f,
    .kd[RATES] = 0.0f,

	.kp[ANGLE] = 0.0f,
	.kp[SPEED] = 0.0f,
	.ki[ANGLE] = 0.0f,
	.ki[SPEED] = 0.0f,
	.kd[ANGLE] = 0.0f,
	.kd[SPEED] = 0.0f,
  }
};

/**
  * @brief  ��̨������ʼ��
  * @param  void 
  * @retval void
  */
void Gimbal_InitConfig(void)
{
	PID_Init(&Yaw_speed_pid,	  //Yaw����ٶȻ�
			  _GimbalParam[YAW].kp[RATES],//kp
			  _GimbalParam[YAW].ki[RATES],//ki
			  _GimbalParam[YAW].kd[RATES],//kd
			  10000,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  30);			      //�����޷�

	PID_Init(&Pitch_speed_pid,	  //Pitch����ٶȻ�
			  _GimbalParam[PIT].kp[RATES],//kp
			  _GimbalParam[PIT].ki[RATES],//ki
			  _GimbalParam[PIT].kd[RATES],//kd
			  10000,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  30);			      //�����޷�

	PID_Init(&Yaw_position_pid,	  //Yaw���λ�û�
			  _GimbalParam[YAW].kp[EULER],//kp
			  _GimbalParam[YAW].ki[EULER],//ki
			  _GimbalParam[YAW].kd[EULER],//kd
			  10000,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  30);			      //�����޷�

	PID_Init(&Pitch_position_pid,	  //Pitch���λ�û�
			  _GimbalParam[PIT].kp[EULER],//kp
			  _GimbalParam[PIT].ki[EULER],//ki
			  _GimbalParam[PIT].kd[EULER],//kd
			  10000,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  30);			      //�����޷�

/*���ݱ�����������е�Ƕ�����PID*/
	PID_Init(&Yaw_speed_encode_pid,	      //Yaw����ٶȻ�
			  _GimbalParam[YAW].kp[SPEED],//kp
			  _GimbalParam[YAW].ki[SPEED],//ki
			  _GimbalParam[YAW].kd[SPEED],//kd
			  10000,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  0);			      //�����޷�

	PID_Init(&Pitch_speed_encode_pid,	  //Pitch����ٶȻ�
			  _GimbalParam[PIT].kp[SPEED],//kp
			  _GimbalParam[PIT].ki[SPEED],//ki
			  _GimbalParam[PIT].kd[SPEED],//kd
			  10000,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  0);			      //�����޷�

	PID_Init(&Yaw_position_encode_pid,	  //Yaw���λ�û�
			  _GimbalParam[YAW].kp[ANGLE],//kp
			  _GimbalParam[YAW].ki[ANGLE],//ki
			  _GimbalParam[YAW].kd[ANGLE],//kd
			  10000,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  0);			      //�����޷�

	PID_Init(&Pitch_position_encode_pid,	  //Pitch���λ�û�
			  _GimbalParam[PIT].kp[ANGLE],//kp
			  _GimbalParam[PIT].ki[ANGLE],//ki
			  _GimbalParam[PIT].kd[ANGLE],//kd
			  10000,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  0);			      //�����޷�


}
