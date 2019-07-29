#include "Driver_Chassis.h"
#include "Driver_Gimbal.h"

#include "Algorithm_Pid.h"

#include "stdlib.h"
#include "time.h"
#include "math.h"

ChassisParam_t _ChassisParam = {

    .Motor[0] = 0,
    .Motor[1] = 0,
    .Motor[2] = 0,
    .Motor[3] = 0,
    .TargetVX = 0,
    .TargetVY = 0,
    .TargetOmega = 0,
    .TargetABSAngle = 0,

//	.MaxWheelSpeed = 4.06f,// (m/s)
    .MaxWheelSpeed = 3.0f,// (m/s)
	.ChassisCurrent[0] = 0,
	.ChassisCurrent[1] = 0,
	.ChassisCurrent[2] = 0,
	.ChassisCurrent[3] = 0,
	.ChassisMaxSumCurrent = 17000,

//	.kp[0] = 4.0f,
//	.kp[1] = 4.0f,
//	.kp[2] = 4.0f,
//	.kp[3] = 4.0f,
//	.kp[4] = 4.0f,

//	.ki[0] = 0.05f,
//	.ki[1] = 0.05f,
//	.ki[2] = 0.05f,
//	.ki[3] = 0.05f,
//	.ki[4] = 0.05f,

//	.kd[0] = 0.0f,
//	.kd[1] = 0.0f,
//	.kd[2] = 0.0f,
//	.kd[3] = 0.0f,
//	.kd[4] = 0.0f

    .kp[0] = 9.0f,
	.kp[1] = 9.0f,
	.kp[2] = 9.0f,
	.kp[3] = 9.0f,
	.kp[4] = 0.5f,

	.ki[0] = 0.45f,
	.ki[1] = 0.45f,
	.ki[2] = 0.45f,
	.ki[3] = 0.45f,
	.ki[4] = 0.01f,

	.kd[0] = 0.0f,
	.kd[1] = 0.0f,
	.kd[2] = 0.0f,
	.kd[3] = 0.0f,
	.kd[4] = 0.0f
 };

/**
  * @brief  ���̲�����ʼ��
  * @param  void 
  * @retval void
  */
void Chassis_InitConfig(void)
{
	PID_Init(&CM_speed_pid[0],	  //1�ŵ���ٶȻ�
			  _ChassisParam.kp[0],//kp
			  _ChassisParam.ki[0],//ki
			  _ChassisParam.kd[0],//kd
			  10.0f,			  //PID������16.1
			  0,				  //����
			  0,				  //���ַ�Χ
			  3);			      //�����޷�

	PID_Init(&CM_speed_pid[1],	  //2�ŵ���ٶȻ�
			  _ChassisParam.kp[1],//kp
			  _ChassisParam.ki[1],//ki
			  _ChassisParam.kd[1],//kd
			  10.0f,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  3);			      //�����޷�

	PID_Init(&CM_speed_pid[2],	  //1�ŵ���ٶȻ�
			  _ChassisParam.kp[2],//kp
			  _ChassisParam.ki[2],//ki
			  _ChassisParam.kd[2],//kd
			  10.0f,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  3);			      //�����޷�

	PID_Init(&CM_speed_pid[3],	  //4�ŵ���ٶȻ�
			  _ChassisParam.kp[3],//kp
			  _ChassisParam.ki[3],//ki
			  _ChassisParam.kd[3],//kd
			  10.0f,			  //PID������
			  0,				  //����
			  0,				  //���ַ�Χ
			  3);			      //�����޷�

	PID_Init(&CM_rotate_pid,	  //���̸���λ�û�
			  _ChassisParam.kp[4],//kp
			  _ChassisParam.ki[4],//ki
			  _ChassisParam.kd[4],//kd
			  2.0f,			  //PID������
			  40,				  //����
			  0,				  //���ַ�Χ
			  3);			      //�����޷�
}

/**
  * @brief  XY�����ٶ�����
  * @param  X�ٶ�
  * @param  Y�ٶ�
  * @retval void
  */
void Chassis_SpeedSet(float XSpeed, float YSpeed, float OmegaSpeed)
{
    XSpeed = XSpeed > _ChassisParam.MaxWheelSpeed? _ChassisParam.MaxWheelSpeed : XSpeed;
    XSpeed = XSpeed < -_ChassisParam.MaxWheelSpeed ? -_ChassisParam.MaxWheelSpeed : XSpeed;

    YSpeed = YSpeed > _ChassisParam.MaxWheelSpeed ? _ChassisParam.MaxWheelSpeed : YSpeed;
    YSpeed = YSpeed < -_ChassisParam.MaxWheelSpeed ? -_ChassisParam.MaxWheelSpeed : YSpeed;

    _ChassisParam.TargetVX = XSpeed;
    _ChassisParam.TargetVY = YSpeed;
	_ChassisParam.TargetOmega = OmegaSpeed;
}

//void Current_Distribution(int16_t *current, uint16_t total_current)
//{
//	int32_t sum = (int32_t)(current[0] + current[1] + current[2] + current[3]);

//	if(sum > total_current)
//	{
//	float k[4] = {(float)current[0]/(float)sum,
//				  (float)current[1]/(float)sum,
//				  (float)current[2]/(float)sum,
//				  (float)current[3]/(float)sum};

//	for(int i = 0; i<4; i++) {
//	current[i] = (int16_t)(total_current * k[i]);
//	}
//	}
//}

void Current_Distribution(int16_t *current, uint16_t total_current)
{
	int32_t sum = (int32_t)(fabsf((float)current[0]) + fabsf((float)current[1]) + fabsf((float)current[2]) + fabsf((float)current[3]));

    float temp = (float)sum/(float)total_current;

	if(temp > 1)
	{
//	float k[4] = {(float)current[0]/(float)sum,
//				  (float)current[1]/(float)sum,
//				  (float)current[2]/(float)sum,
//				  (float)current[3]/(float)sum};

	for(int i = 0; i<4; i++) {
        current[i] /= temp;
	}
	}
}

void Twist_Waist(uint8_t mode)
{
    static int8_t temp = 1 ;
    static uint32_t Random_Absangle = 0 ;
    switch (mode)
    {
        case 1 :
            PID_Calc(&CM_rotate_pid, _GimbalParam[YAW].RealEncodeAngle, YAW_ABSANGLE + temp * 1000, POSITION_PID);
            if (( 1 == temp && _GimbalParam[YAW].RealEncodeAngle > YAW_ABSANGLE + 950 ) || ( -1 == temp && _GimbalParam[YAW].RealEncodeAngle < YAW_ABSANGLE + 950 ))
            {
                temp *= -1;
            }
            break;
        case 2 :
            PID_Calc(&CM_rotate_pid, _GimbalParam[YAW].RealEncodeAngle, temp *( YAW_ABSANGLE + Random_Absangle ), POSITION_PID);
            if (( 1 == temp && _GimbalParam[YAW].RealEncodeAngle > YAW_ABSANGLE + 0.95 * temp * Random_Absangle ) || ( -1 == temp && _GimbalParam[YAW].RealEncodeAngle < YAW_ABSANGLE + 0.95 * temp * Random_Absangle ))
            {   
                temp *= -1;
                Random_Absangle = rand()%801+500;
            }
            break;
        default :
            Random_Absangle = 0;
            break;
    }
}

