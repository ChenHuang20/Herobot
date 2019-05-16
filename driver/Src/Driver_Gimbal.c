#include "Driver_Gimbal.h"

#include "Algorithm_Pid.h"

GimbalParam_t _GimbalParam[2] = {
  {                                   //YAW
    .RealEncodeAngle = 0,             //实际编码器反馈机械角度
    .MiddleEncoderAngle = 0,          //中间位置的机械角度
    .FrameCounter = 0,                //帧率计数器
    .FrameRate = 0,                   //帧率
    .SendCurrent = 0,                //CAN发送报文中的电流值
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
    .RealEncodeAngle = 0,             //实际编码器反馈机械角度
    .MiddleEncoderAngle = 0,          //中间位置的机械角度
    .FrameCounter = 0,                //帧率计数器
    .FrameRate = 0,                   //帧率
    .SendCurrent = 0,                //CAN发送报文中的电流值
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
  * @brief  云台参数初始化
  * @param  void 
  * @retval void
  */
void Gimbal_InitConfig(void)
{
	PID_Init(&Yaw_speed_pid,	  //Yaw电机速度环
			  _GimbalParam[YAW].kp[RATES],//kp
			  _GimbalParam[YAW].ki[RATES],//ki
			  _GimbalParam[YAW].kd[RATES],//kd
			  10000,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  30);			      //积分限幅

	PID_Init(&Pitch_speed_pid,	  //Pitch电机速度环
			  _GimbalParam[PIT].kp[RATES],//kp
			  _GimbalParam[PIT].ki[RATES],//ki
			  _GimbalParam[PIT].kd[RATES],//kd
			  10000,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  30);			      //积分限幅

	PID_Init(&Yaw_position_pid,	  //Yaw电机位置环
			  _GimbalParam[YAW].kp[EULER],//kp
			  _GimbalParam[YAW].ki[EULER],//ki
			  _GimbalParam[YAW].kd[EULER],//kd
			  10000,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  30);			      //积分限幅

	PID_Init(&Pitch_position_pid,	  //Pitch电机位置环
			  _GimbalParam[PIT].kp[EULER],//kp
			  _GimbalParam[PIT].ki[EULER],//ki
			  _GimbalParam[PIT].kd[EULER],//kd
			  10000,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  30);			      //积分限幅

/*依据编码器反馈机械角度做的PID*/
	PID_Init(&Yaw_speed_encode_pid,	      //Yaw电机速度环
			  _GimbalParam[YAW].kp[SPEED],//kp
			  _GimbalParam[YAW].ki[SPEED],//ki
			  _GimbalParam[YAW].kd[SPEED],//kd
			  10000,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  0);			      //积分限幅

	PID_Init(&Pitch_speed_encode_pid,	  //Pitch电机速度环
			  _GimbalParam[PIT].kp[SPEED],//kp
			  _GimbalParam[PIT].ki[SPEED],//ki
			  _GimbalParam[PIT].kd[SPEED],//kd
			  10000,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  0);			      //积分限幅

	PID_Init(&Yaw_position_encode_pid,	  //Yaw电机位置环
			  _GimbalParam[YAW].kp[ANGLE],//kp
			  _GimbalParam[YAW].ki[ANGLE],//ki
			  _GimbalParam[YAW].kd[ANGLE],//kd
			  10000,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  0);			      //积分限幅

	PID_Init(&Pitch_position_encode_pid,	  //Pitch电机位置环
			  _GimbalParam[PIT].kp[ANGLE],//kp
			  _GimbalParam[PIT].ki[ANGLE],//ki
			  _GimbalParam[PIT].kd[ANGLE],//kd
			  10000,			  //PID最大输出
			  0,				  //死区
			  0,				  //积分范围
			  0);			      //积分限幅


}
