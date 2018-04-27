
#include "Driver_Chassis.h"
#include "Algorithm_Pid.h"

ChassisParam_t _ChassisParam = {

    .Motor[0] = 0,
    .Motor[1] = 0,
    .Motor[2] = 0,
    .Motor[3] = 0,
    .TargetVX = 0,
    .TargetVY = 0,
    .TargetOmega = 0,
    .TargetABSAngle = 0,

	.MaxWheelSpeed = 4.06f,// (m/s)
	.ChassisCurrent[0] = 0,
	.ChassisCurrent[1] = 0,
	.ChassisCurrent[2] = 0,
	.ChassisCurrent[3] = 0,
	.ChassisMaxSumCurrent = 17000,

	.kp[0] = 4.0f,
	.kp[1] = 4.0f,
	.kp[2] = 4.0f,
	.kp[3] = 4.0f,

	.ki[0] = 0.05f,
	.ki[1] = 0.05f,
	.ki[2] = 0.05f,
	.ki[3] = 0.05f,

	.kd[0] = 0.0f,
	.kd[1] = 0.0f,
	.kd[2] = 0.0f,
	.kd[3] = 0.0f,
 };

/**
  * @brief  底盘参数初始化
  * @param  void 
  * @retval void
  */
void Chassis_InitConfig(void)
{
	PID_Init(&CM_speed_pid[0],	  //1号电机速度环
			  _ChassisParam.kp[0],//kp
			  _ChassisParam.ki[0],//ki
			  _ChassisParam.kd[0],//kd
			  16.1f,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  3);			      //积分限幅

	PID_Init(&CM_speed_pid[1],	  //2号电机速度环
			  _ChassisParam.kp[1],//kp
			  _ChassisParam.ki[1],//ki
			  _ChassisParam.kd[1],//kd
			  16.1f,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  3);			      //积分限幅

	PID_Init(&CM_speed_pid[2],	  //1号电机速度环
			  _ChassisParam.kp[2],//kp
			  _ChassisParam.ki[2],//ki
			  _ChassisParam.kd[2],//kd
			  16.1f,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  3);			      //积分限幅

	PID_Init(&CM_speed_pid[3],	  //4号电机速度环
			  _ChassisParam.kp[3],//kp
			  _ChassisParam.ki[3],//ki
			  _ChassisParam.kd[3],//kd
			  16.1f,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  3);			      //积分限幅
}

/**
  * @brief  XY方向速度设置
  * @param  X速度
  * @param  Y速度
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

void Current_Distribution(int16_t *current, uint16_t total_current)
{
	int32_t sum = (int32_t)(current[0] + current[1] + current[2] + current[3]);

	if(sum > total_current)
	{
	float k[4] = {(float)current[0]/(float)sum,
				  (float)current[1]/(float)sum,
				  (float)current[2]/(float)sum,
				  (float)current[3]/(float)sum};

	for(int i = 0; i<4; i++) {
	current[i] = (int16_t)(total_current * k[i]);
	}
	}
}

