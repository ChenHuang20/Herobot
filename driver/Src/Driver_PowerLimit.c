
#include "BSP_TIM.h"

#include "Driver_Judge.h"
#include "Driver_PowerLimit.h"
#include "Driver_Chassis.h"
#include "Algorithm_Pid.h"

PowerLimit_t PowerLimit = { 0 };
Increment_t Increment = { 0 };

void PowerLimit_InitConfig(void)
{
	PowerLimit.RemainPower[2] = 60;
	PowerLimit.MaxSpeed = _ChassisParam.MaxWheelSpeed;

	PID_Init(&Power_Limit_pid,	  //4号电机速度环
		  0.5f,//kp
		  0,//ki
		  0,//kd
		  0.875f,			  //PID最大输出
		  0,				  //死区
		  0,				  //积分范围
		  0);			      //积分限幅
}

float MyAbs(float num)
{
	if(num>=0)
		return num;
	else 
		return -num;	
}

/**
  * @brief  防止裁判系统失真
  * @param  void
  * @retval void
  */
void Power_Deal(void)
{
	//_Judge.chassisPower  实时功率值
	//_Judge.chassisPowerBuffer  能量槽剩余能量
	if(_Judge.PowerHeatTick + 20 > Tick || PowerLimit.RemainPower[2] > 20)//保证频率为50Hz
	{
		PowerLimit.Real_Power[2] = _Judge.chassisPower;
		PowerLimit.RemainPower[2] = _Judge.chassisPowerBuffer;
	}
	else
	{
		//数据帧丢失，通过微分关系预测此时功率，为了保证预测功率小于实际功率，取安全系数1.3
		PowerLimit.Real_Power[2] = (2 * PowerLimit.Real_Power[1] - PowerLimit.Real_Power[0]) * 1.3f;
		PowerLimit.RemainPower[2]=(2 * PowerLimit.RemainPower[1] - PowerLimit.RemainPower[0]) / 1.3f;
	}

	PowerLimit.Real_Power[1] = PowerLimit.Real_Power[2];
	PowerLimit.Real_Power[0] = PowerLimit.Real_Power[1];
	
	PowerLimit.RemainPower[1] = PowerLimit.RemainPower[2];
	PowerLimit.RemainPower[0] = PowerLimit.RemainPower[1];
}





/**
  * @brief  防止运动轨迹失真
  * @param  轮子速度
  * @retval void
  */
void Moving_Trial(float *MotoSpeed)
{
//这几行代码是用来防止某个轮子的速度超过最大的给定速度
//为了使运动轨迹保持和操控的方向一样
//如果有一个轮子超过了最大速度值，等比例缩放四个轮子的速度
	PowerLimit.Zoom = 1;

	for(PowerLimit.ZoomCount = 0; PowerLimit.ZoomCount < 4; PowerLimit.ZoomCount++)
	{
		if(MyAbs(MotoSpeed[PowerLimit.ZoomCount])>PowerLimit.MaxSpeed)
		{
			PowerLimit.ZoomTemp=MyAbs(MotoSpeed[PowerLimit.ZoomCount])/PowerLimit.MaxSpeed;
			if(PowerLimit.ZoomTemp>PowerLimit.Zoom)
				PowerLimit.Zoom=PowerLimit.ZoomTemp;
		}
	}
	if(PowerLimit.Zoom != 1)
	{
		for(PowerLimit.ZoomCount=0;PowerLimit.ZoomCount<4;PowerLimit.ZoomCount++)
		{
		MotoSpeed[PowerLimit.ZoomCount] = MotoSpeed[PowerLimit.ZoomCount] / PowerLimit.Zoom;
		}
		PowerLimit.ZoomTemp = 1;
		PowerLimit.Zoom = 1;
	}

}

///*****************************************************************电机增量处理任务**************************************************************************************/

//int32_t Real_Speed[7] = { 0 };//发送给PID的真实速度值

//int32_t Motor_Increment_Deal(uint8_t Motor_ID, int32_t Set_Val, float Increment)
//{
//	if((Set_Val-Real_Speed[Motor_ID - 1]) >= 0)//当设定值大于实际值，累加，否则累减
//	{
//		Real_Speed[Motor_ID - 1] += Increment;
//		if(Real_Speed[Motor_ID - 1] > Set_Val)
//			Real_Speed[Motor_ID - 1] = Set_Val;
//	}
//	else
//	{
//		Real_Speed[Motor_ID - 1] -= Increment;
//		if(Real_Speed[Motor_ID - 1] < Set_Val)
//			Real_Speed[Motor_ID - 1] = Set_Val;
//	}		
//	return Real_Speed[Motor_ID - 1];
//}

//void Motor_Increment_Self_Deal(float* MotoSpeed)
//{
//	Increment.MaxBias=0;
//	//记录每个轮子的速度差
//	Increment.Bias[1] = MyAbs(MotoSpeed[0]-_ChassisParam.Motor[0].RealSpeed);
//	Increment.Bias[2] = MyAbs(MotoSpeed[1]-_ChassisParam.Motor[0].RealSpeed);
//	Increment.Bias[3] = MyAbs(MotoSpeed[2]-_ChassisParam.Motor[0].RealSpeed);
//	Increment.Bias[4] = MyAbs(MotoSpeed[3]-_ChassisParam.Motor[0].RealSpeed);
//	for(Increment.ZoomCount = 1; Increment.ZoomCount <= 4;Increment.ZoomCount++)
//	{
//		if(Increment.MaxBias < Increment.Bias[Increment.ZoomCount])
//		{
//			//记录轮子最大速度差
//			Increment.MaxBias = Increment.Bias[Increment.ZoomCount];
//			//记录有最大速度差的轮子
//			Increment.MotorID = Increment.ZoomCount;
//		}
//	}

//	if(MyAbs(MyAbs(MotoSpeed[Increment.MotorID])-Increment.MaxBias)<3600)
//		Increment.Increment[0]=AccelK1*MyAbs(MyAbs(MotoSpeed[Increment.MotorID])-Increment.MaxBias)+AccelB1;    
//		//期望速度-最大偏差=当前速度，通过当前速度获取当前的斜率
//	else
//		Increment.Increment[0]=AccelK2*MyAbs(MyAbs(MotoSpeed[Increment.MotorID])-Increment.MaxBias)+AccelB2;    
//		//分段处理加速度，启动的时候加速度小，中间加速度大，接近最大速度加速度小

//	//	Increment.Increment[0]=-0.01f*MyAbs(MyAbs(CMSpeed[Increment.MotorID])-Increment.MaxBias)+100;         
//		//速度越大加速度越小，这样启动快，但是操作手感不怎么样
//	for(Increment.ZoomCount=1;Increment.ZoomCount<=4;Increment.ZoomCount++)
//		{
//			Increment.Increment[Increment.ZoomCount]=Increment.Increment[0] * Increment.Bias[Increment.ZoomCount] / (Increment.MaxBias);
//			VAL_LIMIT(Increment.Increment[Increment.ZoomCount],0,MaxSpeedIncrement);
//		}
//}
