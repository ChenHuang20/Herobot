
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

	PID_Init(&Power_Limit_pid,	  //4�ŵ���ٶȻ�
		  0.5f,//kp
		  0,//ki
		  0,//kd
		  0.875f,			  //PID������
		  0,				  //����
		  0,				  //���ַ�Χ
		  0);			      //�����޷�
}

float MyAbs(float num)
{
	if(num>=0)
		return num;
	else 
		return -num;	
}

/**
  * @brief  ��ֹ����ϵͳʧ��
  * @param  void
  * @retval void
  */
void Power_Deal(void)
{
	//_Judge.chassisPower  ʵʱ����ֵ
	//_Judge.chassisPowerBuffer  ������ʣ������
	if(_Judge.PowerHeatTick + 20 > Tick || PowerLimit.RemainPower[2] > 20)//��֤Ƶ��Ϊ50Hz
	{
		PowerLimit.Real_Power[2] = _Judge.chassisPower;
		PowerLimit.RemainPower[2] = _Judge.chassisPowerBuffer;
	}
	else
	{
		//����֡��ʧ��ͨ��΢�ֹ�ϵԤ���ʱ���ʣ�Ϊ�˱�֤Ԥ�⹦��С��ʵ�ʹ��ʣ�ȡ��ȫϵ��1.3
		PowerLimit.Real_Power[2] = (2 * PowerLimit.Real_Power[1] - PowerLimit.Real_Power[0]) * 1.3f;
		PowerLimit.RemainPower[2]=(2 * PowerLimit.RemainPower[1] - PowerLimit.RemainPower[0]) / 1.3f;
	}

	PowerLimit.Real_Power[1] = PowerLimit.Real_Power[2];
	PowerLimit.Real_Power[0] = PowerLimit.Real_Power[1];
	
	PowerLimit.RemainPower[1] = PowerLimit.RemainPower[2];
	PowerLimit.RemainPower[0] = PowerLimit.RemainPower[1];
}





/**
  * @brief  ��ֹ�˶��켣ʧ��
  * @param  �����ٶ�
  * @retval void
  */
void Moving_Trial(float *MotoSpeed)
{
//�⼸�д�����������ֹĳ�����ӵ��ٶȳ������ĸ����ٶ�
//Ϊ��ʹ�˶��켣���ֺͲٿصķ���һ��
//�����һ�����ӳ���������ٶ�ֵ���ȱ��������ĸ����ӵ��ٶ�
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

///*****************************************************************���������������**************************************************************************************/

//int32_t Real_Speed[7] = { 0 };//���͸�PID����ʵ�ٶ�ֵ

//int32_t Motor_Increment_Deal(uint8_t Motor_ID, int32_t Set_Val, float Increment)
//{
//	if((Set_Val-Real_Speed[Motor_ID - 1]) >= 0)//���趨ֵ����ʵ��ֵ���ۼӣ������ۼ�
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
//	//��¼ÿ�����ӵ��ٶȲ�
//	Increment.Bias[1] = MyAbs(MotoSpeed[0]-_ChassisParam.Motor[0].RealSpeed);
//	Increment.Bias[2] = MyAbs(MotoSpeed[1]-_ChassisParam.Motor[0].RealSpeed);
//	Increment.Bias[3] = MyAbs(MotoSpeed[2]-_ChassisParam.Motor[0].RealSpeed);
//	Increment.Bias[4] = MyAbs(MotoSpeed[3]-_ChassisParam.Motor[0].RealSpeed);
//	for(Increment.ZoomCount = 1; Increment.ZoomCount <= 4;Increment.ZoomCount++)
//	{
//		if(Increment.MaxBias < Increment.Bias[Increment.ZoomCount])
//		{
//			//��¼��������ٶȲ�
//			Increment.MaxBias = Increment.Bias[Increment.ZoomCount];
//			//��¼������ٶȲ������
//			Increment.MotorID = Increment.ZoomCount;
//		}
//	}

//	if(MyAbs(MyAbs(MotoSpeed[Increment.MotorID])-Increment.MaxBias)<3600)
//		Increment.Increment[0]=AccelK1*MyAbs(MyAbs(MotoSpeed[Increment.MotorID])-Increment.MaxBias)+AccelB1;    
//		//�����ٶ�-���ƫ��=��ǰ�ٶȣ�ͨ����ǰ�ٶȻ�ȡ��ǰ��б��
//	else
//		Increment.Increment[0]=AccelK2*MyAbs(MyAbs(MotoSpeed[Increment.MotorID])-Increment.MaxBias)+AccelB2;    
//		//�ֶδ�����ٶȣ�������ʱ����ٶ�С���м���ٶȴ󣬽ӽ�����ٶȼ��ٶ�С

//	//	Increment.Increment[0]=-0.01f*MyAbs(MyAbs(CMSpeed[Increment.MotorID])-Increment.MaxBias)+100;         
//		//�ٶ�Խ����ٶ�ԽС�����������죬���ǲ����ָв���ô��
//	for(Increment.ZoomCount=1;Increment.ZoomCount<=4;Increment.ZoomCount++)
//		{
//			Increment.Increment[Increment.ZoomCount]=Increment.Increment[0] * Increment.Bias[Increment.ZoomCount] / (Increment.MaxBias);
//			VAL_LIMIT(Increment.Increment[Increment.ZoomCount],0,MaxSpeedIncrement);
//		}
//}
