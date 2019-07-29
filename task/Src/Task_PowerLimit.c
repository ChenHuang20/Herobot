
#include "Handle.h"

#include <math.h>

#include "Driver_PowerLimit.h"
#include "Driver_Chassis.h"

#include "Task_PowerLimit.h"

#include "Algorithm_Pid.h"

/**
  * @brief  ���̹�����������
  * @param  unused
  * @retval void
  */
void Task_PowerLimit(void * pvParameters )
{
	_StackSurplus.PowerLimit = uxTaskGetStackHighWaterMark(NULL);

    while(1)
    {
		_Tick.PowerLimit++;

//		Power_Deal();//��ֹ��������ʧ�洦��

		//����ʣ�������������Ƶ�����ٶ�ֵ
		if(PowerLimit.RemainPower[2]>=40)
		{
			PowerLimit.MaxSpeed = _ChassisParam.MaxWheelSpeed;
			PowerLimit.Flag = 0;
			PowerLimit.pidTemp = 0;
		}
		else if(PowerLimit.RemainPower[2] > 10 && PowerLimit.RemainPower[2] < 40)
		{
			PID_Calc(&Power_Limit_pid, PowerLimit.RemainPower[2], 40, POSITION_PID);//�趨����ֵ-ʵ�ʹ���ֵ
			PowerLimit.RealSpeed = fabsf(_ChassisParam.TargetVX) + fabsf(_ChassisParam.TargetVY) + fabsf(_ChassisParam.TargetOmega);
			PowerLimit.RealSpeed = PowerLimit.RealSpeed > PowerLimit.MaxSpeed ? PowerLimit.MaxSpeed : PowerLimit.RealSpeed;
			if(PowerLimit.Flag == 0)
			{
				PowerLimit.Flag = 1;
				PowerLimit.v = PowerLimit.RealSpeed;
			}
			if(PowerLimit.RealSpeed > PowerLimit.v)
				PowerLimit.MaxSpeed = PowerLimit.v*(1-(Power_Limit_pid.output-PowerLimit.pidTemp)/(1.8f-PowerLimit.pidTemp));
			else
			{
				PowerLimit.MaxSpeed = PowerLimit.RealSpeed*(1-(Power_Limit_pid.output-PowerLimit.pidTemp)/(1.8f-PowerLimit.pidTemp));
				if(PowerLimit.MaxSpeed <= (PowerLimit.RealSpeed - 0.95f))
				{
					PowerLimit.v = PowerLimit.RealSpeed - 0.95f;
					PowerLimit.v = PowerLimit.v <= 0 ? 0 : PowerLimit.v;
					PowerLimit.pidTemp += (1.8f-PowerLimit.pidTemp)*(1/(PowerLimit.RealSpeed));
				}
			}
            if(PowerLimit.RealSpeed > PowerLimit.v)
				PowerLimit.MaxSpeed = PowerLimit.v*(1-(Power_Limit_pid.output-PowerLimit.pidTemp)/(0.875f-PowerLimit.pidTemp));
			else
			{
				PowerLimit.MaxSpeed = PowerLimit.RealSpeed*(1-(Power_Limit_pid.output-PowerLimit.pidTemp)/(0.875f-PowerLimit.pidTemp));
				if(PowerLimit.MaxSpeed <= (PowerLimit.RealSpeed - 1.0f))
				{
					PowerLimit.v = PowerLimit.RealSpeed - 1.0f;
					PowerLimit.v = PowerLimit.v <= 0 ? 0 : PowerLimit.v;
					PowerLimit.pidTemp += (0.875f-PowerLimit.pidTemp)*(1/(PowerLimit.RealSpeed));
				}
			}
            
		}
		else
		{
			PowerLimit.Cut = ON;
		}
		//��ȡʣ��ջ��С
		_StackSurplus.PowerLimit = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelay(20);
    }
}


